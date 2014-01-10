/*
 * tvp5150 - Texas Instruments TVP5150A/AM1 video decoder driver
 *
 * Copyright (c) 2012 Plextek Ltd (aps@plextek.co.uk)
 * Copyright (c) 2005,2006 Mauro Carvalho Chehab (mchehab@infradead.org)
 * This code is placed under the terms of the GNU General Public License v2
 */

//#define DEBUG
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <media/v4l2-device.h>
#include <media/tvp5150.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"
#include "tvp5150_reg.h"

/* **************************************************************************
 * Module Information
 * *************************************************************************/

MODULE_DESCRIPTION("Texas Instruments TVP5150A video decoder driver");
MODULE_AUTHOR("Adam Sutton (based on Mauro Carvalho Chehab)");
MODULE_LICENSE("GPL");

#define DEBUG
#ifdef DEBUG
	#define TRACE() pr_err("\ntvp5150: %s\n", __func__);
#else
	#define TRACE() (void)0
#endif

/* Worker thread */
static void tvp5150_work_handler ( struct work_struct *w );
static struct workqueue_struct *tvp5150_wq = NULL;
static DECLARE_DELAYED_WORK(tvp5150_work, tvp5150_work_handler);

/* **************************************************************************
 * Globals
 * *************************************************************************/

/* Sensor data */
struct sensor {
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client      *i2c_client;
	struct v4l2_routing    route;
	v4l2_std_id            std_id;
	bool                   on;
} tvp5150_data;

static struct fsl_mxc_tvin_platform_data *plat_data;

/* **************************************************************************
 * I2C read/write
 * *************************************************************************/

/* Register setting */
struct i2c_reg_value {
	u8 reg;
	u8 val;
};

/* Read register */
static int tvp5150_read ( struct i2c_client *c, u8 reg )
{
	int val;

	val = i2c_smbus_read_byte_data(c, reg);
	if ( val < 0 ) {
		dev_err(&c->dev, "rd reg err, reg=%02x, err=%d\n", reg, val);
		return -1;
	}

	dev_dbg(&c->dev, "rd reg %02x = %02x\n", reg, val);
	return val;
}

/* Write register */
static int tvp5150_write ( struct i2c_client *c, u8 reg, u8 val )
{
	int err;

	dev_dbg(&c->dev, "wr reg %02x = %02x\n", reg, val);
	
	if ( (err = i2c_smbus_write_byte_data(c, reg, val)) < 0 ) {
		dev_err(&c->dev, "wr reg err, reg=%02x val=%02x, err=%d\n", reg, val, err);
		return -1;
	}
	return 0;
}

/* Write array */
static int tvp5150_write_array(struct i2c_client *c, const struct i2c_reg_value *regs)
{
	int err = 0;
	while ( !err && (regs->reg != 0xff) ) {
		err = tvp5150_write(c, regs->reg, regs->val);
		regs++;
	}
	return err;
}

/* **************************************************************************
 * I2C debug
 * *************************************************************************/

/* Register dump */
static void tvp5150_dump(struct i2c_client *c, char* pre, u8 beg, u8 end, int line)
{
	int i = 0;
	while ( beg <= end ) {
		if ( (i % line) == 0 ) {
			if ( i > 0 ) printk("\n");
			printk("tvp5150: %s reg 0x%02x = ", pre, beg);
		}
		printk("%02x ", tvp5150_read(c, beg));
		beg++;
		i++;
	}
	printk("\n");
}

/* Status dump */
static int tvp5150_status ( struct i2c_client *c, int full )
{
	if ( full ) {
		printk("tvp5150: Video input source selection #1 = 0x%02x\n",
			tvp5150_read(c, TVP5150_VD_IN_SRC_SEL_1));
		printk("tvp5150: Analog channel controls = 0x%02x\n",
			tvp5150_read(c, TVP5150_ANAL_CHL_CTL));
		printk("tvp5150: Operation mode controls = 0x%02x\n",
			tvp5150_read(c, TVP5150_OP_MODE_CTL));
		printk("tvp5150: Miscellaneous controls = 0x%02x\n",
			tvp5150_read(c, TVP5150_MISC_CTL));
		printk("tvp5150: Autoswitch mask= 0x%02x\n",
			tvp5150_read(c, TVP5150_AUTOSW_MSK));
		printk("tvp5150: Color killer threshold control = 0x%02x\n",
			tvp5150_read(c, TVP5150_COLOR_KIL_THSH_CTL));
		printk("tvp5150: Luminance processing controls #1 #2 and #3 = %02x %02x %02x\n",
			tvp5150_read(c, TVP5150_LUMA_PROC_CTL_1),
			tvp5150_read(c, TVP5150_LUMA_PROC_CTL_2),
			tvp5150_read(c, TVP5150_LUMA_PROC_CTL_3));
		printk("tvp5150: Brightness control = 0x%02x\n",
			tvp5150_read(c, TVP5150_BRIGHT_CTL));
		printk("tvp5150: Color saturation control = 0x%02x\n",
			tvp5150_read(c, TVP5150_SATURATION_CTL));
		printk("tvp5150: Hue control = 0x%02x\n",
			tvp5150_read(c, TVP5150_HUE_CTL));
		printk("tvp5150: Contrast control = 0x%02x\n",
			tvp5150_read(c, TVP5150_CONTRAST_CTL));
		printk("tvp5150: Outputs and data rates select = 0x%02x\n",
			tvp5150_read(c, TVP5150_DATA_RATE_SEL));
		printk("tvp5150: Configuration shared pins = 0x%02x\n",
			tvp5150_read(c, TVP5150_CONF_SHARED_PIN));
		printk("tvp5150: Active video cropping start = 0x%02x%02x\n",
			tvp5150_read(c, TVP5150_ACT_VD_CROP_ST_MSB),
			tvp5150_read(c, TVP5150_ACT_VD_CROP_ST_LSB));
		printk("tvp5150: Active video cropping stop  = 0x%02x%02x\n",
			tvp5150_read(c, TVP5150_ACT_VD_CROP_STP_MSB),
			tvp5150_read(c, TVP5150_ACT_VD_CROP_STP_LSB));
		printk("tvp5150: Genlock/RTC = 0x%02x\n",
			tvp5150_read(c, TVP5150_GENLOCK));
		printk("tvp5150: Horizontal sync start = 0x%02x\n",
			tvp5150_read(c, TVP5150_HORIZ_SYNC_START));
		printk("tvp5150: Vertical blanking start = 0x%02x\n",
			tvp5150_read(c, TVP5150_VERT_BLANKING_START));
		printk("tvp5150: Vertical blanking stop = 0x%02x\n",
			tvp5150_read(c, TVP5150_VERT_BLANKING_STOP));
		printk("tvp5150: Chrominance processing control #1 and #2 = %02x %02x\n",
			tvp5150_read(c, TVP5150_CHROMA_PROC_CTL_1),
			tvp5150_read(c, TVP5150_CHROMA_PROC_CTL_2));
		printk("tvp5150: Interrupt reset register B = 0x%02x\n",
			tvp5150_read(c, TVP5150_INT_RESET_REG_B));
		printk("tvp5150: Interrupt enable register B = 0x%02x\n",
			tvp5150_read(c, TVP5150_INT_ENABLE_REG_B));
		printk("tvp5150: Interrupt configuration register B = 0x%02x\n",
			tvp5150_read(c, TVP5150_INTT_CONFIG_REG_B));
		printk("tvp5150: Video standard = 0x%02x\n",
			tvp5150_read(c, TVP5150_VIDEO_STD));
		printk("tvp5150: Chroma gain factor: Cb=0x%02x Cr=0x%02x\n",
			tvp5150_read(c, TVP5150_CB_GAIN_FACT),
			tvp5150_read(c, TVP5150_CR_GAIN_FACTOR));
		printk("tvp5150: Macrovision on counter = 0x%02x\n",
			tvp5150_read(c, TVP5150_MACROVISION_ON_CTR));
		printk("tvp5150: Macrovision off counter = 0x%02x\n",
			tvp5150_read(c, TVP5150_MACROVISION_OFF_CTR));
		printk("tvp5150: ITU-R BT.656.%d timing(TVP5150AM1 only)\n",
			(tvp5150_read(c, TVP5150_REV_SELECT) & 1) ? 3 : 4);
		printk("tvp5150: Device ID = %02x%02x\n",
			tvp5150_read(c, TVP5150_MSB_DEV_ID),
			tvp5150_read(c, TVP5150_LSB_DEV_ID));
		printk("tvp5150: ROM version = (hex) %02x.%02x\n",
			tvp5150_read(c, TVP5150_ROM_MAJOR_VER),
			tvp5150_read(c, TVP5150_ROM_MINOR_VER));
		printk("tvp5150: Vertical line count = 0x%02x%02x\n",
			tvp5150_read(c, TVP5150_VERT_LN_COUNT_MSB),
			tvp5150_read(c, TVP5150_VERT_LN_COUNT_LSB));
		printk("tvp5150: Interrupt status register B = 0x%02x\n",
			tvp5150_read(c, TVP5150_INT_STATUS_REG_B));
		printk("tvp5150: Interrupt active register B = 0x%02x\n",
			tvp5150_read(c, TVP5150_INT_ACTIVE_REG_B));
	} /* if ( full ) */

	printk("tvp5150: Status regs #1 to #5 = %02x %02x %02x %02x %02x\n",
		tvp5150_read(c, TVP5150_STATUS_REG_1),
		tvp5150_read(c, TVP5150_STATUS_REG_2),
		tvp5150_read(c, TVP5150_STATUS_REG_3),
		tvp5150_read(c, TVP5150_STATUS_REG_4),
		tvp5150_read(c, TVP5150_STATUS_REG_5));
	if ( full ) {
		tvp5150_dump(c, "Teletext filter 1",   TVP5150_TELETEXT_FIL1_INI, TVP5150_TELETEXT_FIL1_END, 8);
		tvp5150_dump(c, "Teletext filter 2",   TVP5150_TELETEXT_FIL2_INI, TVP5150_TELETEXT_FIL2_END, 8);

		printk("tvp5150: Teletext filter enable = 0x%02x\n",
			tvp5150_read(c, TVP5150_TELETEXT_FIL_ENA));
		printk("tvp5150: Interrupt status register A = 0x%02x\n",
			tvp5150_read(c, TVP5150_INT_STATUS_REG_A));
		printk("tvp5150: Interrupt enable register A = 0x%02x\n",
			tvp5150_read(c, TVP5150_INT_ENABLE_REG_A));
		printk("tvp5150: Interrupt configuration = 0x%02x\n",
			tvp5150_read(c, TVP5150_INT_CONF));
		printk("tvp5150: VDP status register = 0x%02x\n",
			tvp5150_read(c, TVP5150_VDP_STATUS_REG));
		printk("tvp5150: FIFO word count = 0x%02x\n",
			tvp5150_read(c, TVP5150_FIFO_WORD_COUNT));
		printk("tvp5150: FIFO interrupt threshold = 0x%02x\n",
			tvp5150_read(c, TVP5150_FIFO_INT_THRESHOLD));
		printk("tvp5150: FIFO reset = 0x%02x\n",
			tvp5150_read(c, TVP5150_FIFO_RESET));
		printk("tvp5150: Line number interrupt = 0x%02x\n",
			tvp5150_read(c, TVP5150_LINE_NUMBER_INT));
		printk("tvp5150: Pixel alignment register = 0x%02x%02x\n",
			tvp5150_read(c, TVP5150_PIX_ALIGN_REG_HIGH),
			tvp5150_read(c, TVP5150_PIX_ALIGN_REG_LOW));
		printk("tvp5150: FIFO output control = 0x%02x\n",
			tvp5150_read(c, TVP5150_FIFO_OUT_CTRL));
		printk("tvp5150: Full field enable = 0x%02x\n",
			tvp5150_read(c, TVP5150_FULL_FIELD_ENA));
		printk("tvp5150: Full field mode register = 0x%02x\n",
			tvp5150_read(c, TVP5150_FULL_FIELD_MODE_REG));
			tvp5150_dump(c, "CC   data",   TVP5150_CC_DATA_INI, TVP5150_CC_DATA_END, 8);
		tvp5150_dump(c, "WSS  data",   TVP5150_WSS_DATA_INI, TVP5150_WSS_DATA_END, 8);
		tvp5150_dump(c, "VPS  data",   TVP5150_VPS_DATA_INI, TVP5150_VPS_DATA_END, 8);
		tvp5150_dump(c, "VITC data",   TVP5150_VITC_DATA_INI, TVP5150_VITC_DATA_END, 10);
		tvp5150_dump(c, "Line mode",   TVP5150_LINE_MODE_INI, TVP5150_LINE_MODE_END, 8);
	} /* if ( full ) */

	return 0;
}

/****************************************************************************
 * I2C register settings
 ****************************************************************************/

/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_enable[] = {
	{
		TVP5150_MISC_CTL, 0x09
	},
	{
		0xff, 0xff
	}
};

/****************************************************************************
 * Basic Functions
 ****************************************************************************/

/* Select the MUX */
static inline void tvp5150_selmux ( struct sensor *s )
{
	unsigned char val;
	int opmode = 0;
	int input  = 0;
	struct i2c_client *c = s->i2c_client;

	/* Switch on input */
	switch ( s->route.input ) {
	case TVP5150_COMPOSITE1:
		input |= 2;
	/* fall through */
	case TVP5150_COMPOSITE0:
		opmode = 0x30; // TV mode
		break;
	case TVP5150_SVIDEO:
	/* fall through */
	default:
		input |= 1;
		opmode = 0; // Auto mode
		break;
	}
	dev_dbg(&c->dev, "video route in=%i, out=%i => in=%i, opmode=%i\n",
			s->route.input, s->route.output, input, opmode);
          
	/* Select */
	tvp5150_write(c, TVP5150_OP_MODE_CTL,     opmode);
	tvp5150_write(c, TVP5150_VD_IN_SRC_SEL_1, input);

	/* Svideo should enable YCrCb output and disable GPCL output
	 * For Composite and TV, it should be the reverse
	 */
	val = tvp5150_read(c, TVP5150_MISC_CTL);
	if ( s->route.input == TVP5150_SVIDEO )
		val = (val & ~0x40) | 0x10;
	else
		val = (val & ~0x10) | 0x40;

	tvp5150_write(c, TVP5150_MISC_CTL, val);
}

static int tvp5150_set_std ( struct sensor *s, v4l2_std_id std )
{
	int fmt = 0;
	struct i2c_client *c = s->i2c_client;

	/* Store */
	s->std_id = std;

	/* First tests should be against specific std */
	if (std == V4L2_STD_ALL) {
		fmt = 0;  /* Autodetect mode */
	} else if (std & V4L2_STD_NTSC_443) {
		fmt = 0xa;
	} else if (std & V4L2_STD_PAL_M) {
		fmt = 0x6;
	} else if (std & (V4L2_STD_PAL_N | V4L2_STD_PAL_Nc)) {
		fmt = 0x8;
	} else {
		/* Then, test against generic ones */
		if (std & V4L2_STD_NTSC)
			fmt = 0x2;
		else if (std & V4L2_STD_PAL)
			fmt = 0x4;
		else if (std & V4L2_STD_SECAM)
			fmt = 0xc;
	}

	dev_dbg(&c->dev, "set video std reg to %d\n", fmt);
	tvp5150_write(c, TVP5150_VIDEO_STD, fmt);
	return 0;
}

static int tvp5150_get_std ( struct sensor *s, v4l2_std_id *std )
{
	struct i2c_client *c = s->i2c_client;
	int sta1, sta5;

	/* Default to unknown */
	*std = V4L2_STD_UNKNOWN;

	/* Read status registers */
	sta1 = tvp5150_read(c, TVP5150_STATUS_REG_1) & 0xFF;
	sta5 = tvp5150_read(c, TVP5150_STATUS_REG_5) & 0xFF;

	/* Not locked */
	if ( (sta1 & 0x1e) != 0xe ) return 0;
	if ( (sta5 & 0x01) != 0x1 ) return 0;

	sta5 = (sta5 >> 1) & 0x7;

	/* NTSC */
	if(sta5 == 0x0 ) 
		*std = V4L2_STD_NTSC_M;
	else if(sta5 == 0x4)
		*std = V4L2_STD_NTSC_443;
	/* PAL */
	else if(sta5 == 0x1) 
		*std = V4L2_STD_PAL_B | V4L2_STD_PAL_G | V4L2_STD_PAL_H | V4L2_STD_PAL_I | V4L2_STD_PAL_N;
	else if(sta5 == 0x2 )
		*std = V4L2_STD_PAL_M;
	else if(sta5 == 0x3 )
		*std = V4L2_STD_PAL_N;
	/* SECAM */
	else if(sta5 == 0x5)
		*std = V4L2_STD_SECAM;

	return 0;
}

/* Reset the chip */
static int tvp5150_reset (struct sensor *s)
{
	u8 maj_rom, min_rom;
	struct i2c_client *c = s->i2c_client;

	/* Get ROM version */
	maj_rom = tvp5150_read(c, TVP5150_ROM_MAJOR_VER);
	min_rom = tvp5150_read(c, TVP5150_ROM_MINOR_VER);
	dev_dbg(&c->dev, "rom version %d.%d\n", maj_rom, min_rom);

	/* TVP5150AM1 */
	if ( maj_rom == 4 && min_rom == 0 ) {
		dev_dbg(&c->dev, "tvp5150am1 detected\n");
		tvp5150_write(c, TVP5150_REV_SELECT, 0);
	/* TVP5150A */
	} else {
		dev_dbg(&c->dev, "tvp5150a detected\n");
	}

	/* Initialiss TVP5150 to stream enabled values */
	tvp5150_write_array(c, tvp5150_init_enable);

	/* Select input */
	tvp5150_selmux(s);

	/* Ready */
	tvp5150_set_std(s, s->std_id);

	return 0;
}

/****************************************************************************
 * Worker Thread
 ****************************************************************************/

static void tvp5150_work_handler ( struct work_struct *w )
{
	tvp5150_status(tvp5150_data.i2c_client, 0);
	queue_delayed_work(tvp5150_wq, &tvp5150_work, 1*HZ);
}

/****************************************************************************
 * IOCTls
 ****************************************************************************/

/* Reset the chip */
#if 0
static int ioctl_reset ( struct v4l2_int_device *d, void *arg )
{
	struct sensor *s = (struct sensor*)d->priv;
	TRACE();
	return tvp5150_reset(s);
}
#endif

/* Set the video routing */
static int ioctl_s_video_routing(struct v4l2_int_device *d, struct v4l2_routing *route)
{
	struct sensor *s = (struct sensor*)d->priv;
	TRACE();
	s->route = *route;
	tvp5150_selmux(s);
	return 0;
}

/* Set STD */
static int ioctl_s_std(struct v4l2_int_device *d, v4l2_std_id *std)
{
	struct sensor *s = (struct sensor*)d->priv;
	TRACE();

	return tvp5150_set_std(s, *std);
}

/* Get STD */
static int ioctl_g_std(struct v4l2_int_device *d, v4l2_std_id *std)
{
	struct sensor *s = (struct sensor*)d->priv;
	TRACE();

	return tvp5150_get_std(s, std);
}

/* Log status */
#if 0
static int ioctl_log_status ( struct v4l2_int_device *d, void *arg )
{
	struct sensor     *s = (struct sensor*)d->priv;
	struct i2c_client *c = s->i2c_client;
	TRACE();
	tvp5150_status(c, 1);
	return 0;
}
#endif

/* Get chip ident */
static int ioctl_g_chip_ident(struct v4l2_int_device *d, struct v4l2_dbg_chip_ident *chip)
{
	int rev;
	struct sensor     *s = (struct sensor*)d->priv;
	struct i2c_client *c = s->i2c_client;
	TRACE();

	/* Get revision */
	rev = tvp5150_read(c, TVP5150_ROM_MAJOR_VER) << 8 | tvp5150_read(c, TVP5150_ROM_MINOR_VER);

	/* Store data */
	return v4l2_chip_ident_i2c_client(c, chip, V4L2_IDENT_TVP5150, rev);
}

/* Get interface parameter */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	TRACE();
	if ( !s ) {
		pr_err("ERROR!! no slave device set!\n");
		return -1;
	}

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
	p->if_type             = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode        = V4L2_IF_TYPE_BT656_MODE_BT_8BIT;

	return 0;
}

/* Get format */
static int ioctl_g_fmt_cap(struct v4l2_int_device *d, struct v4l2_format *f)
{
	TRACE();

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
		f->fmt.pix.priv        = 1; // TV device
		break;
	default:
		break;
	}

	return 0;
}

/* Set the power state of the device */
static int ioctl_s_power(struct v4l2_int_device *d, int p)
{
	printk("\nEnter func: %s, p = %d\n", __func__, p);
	
	/* Power down */
	if ( !p ) {
	/* Power up */
	} else {
		/* Setup */
		tvp5150_reset(&tvp5150_data);
	}

	return 0;
}

/****************************************************************************
 * V4L2 device
 ****************************************************************************/

/* IOCTL list */
static struct v4l2_int_ioctl_desc tvp5150_ioctl_desc[] = {
	{vidioc_int_s_video_routing_num,  (v4l2_int_ioctl_func*)ioctl_s_video_routing},
	{vidioc_int_s_std_num,            (v4l2_int_ioctl_func*)ioctl_s_std},
	{vidioc_int_querystd_num,         (v4l2_int_ioctl_func*)ioctl_g_std},
	{vidioc_int_g_chip_ident_num,     (v4l2_int_ioctl_func*)ioctl_g_chip_ident},
	{vidioc_int_g_ifparm_num,         (v4l2_int_ioctl_func*)ioctl_g_ifparm},
	{vidioc_int_g_fmt_cap_num,        (v4l2_int_ioctl_func*)ioctl_g_fmt_cap},
	{vidioc_int_s_power_num,          (v4l2_int_ioctl_func*)ioctl_s_power},
};

/* int slave */
static struct v4l2_int_slave tvp5150_slave = {
	.ioctls     = tvp5150_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(tvp5150_ioctl_desc),
};

/* int device */
static struct v4l2_int_device tvp5150_int_device = {
	.module = THIS_MODULE,
	.name   = "tvp5150",
	.type   = v4l2_int_type_slave,
	.u = {
		.slave = &tvp5150_slave,
	},
};

/****************************************************************************
 * I2C driver
 ****************************************************************************/

static int tvp5150_probe(struct i2c_client *c, const struct i2c_device_id *id)
{
	int err;
	int devid;
	TRACE();

	/* Check I2C support */
	if (!i2c_check_functionality(c->adapter, I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE_DATA)) {
		dev_err(&c->dev, "I2C lacks required features\n");
		return -EIO;
	}

	/* Reset chip */
	plat_data = c->dev.platform_data;
	if (plat_data->io_init)
		plat_data->io_init();
	msleep(100);
  
	/* Check I2C device */
	devid = (tvp5150_read(c, TVP5150_MSB_DEV_ID) << 8) | tvp5150_read(c, TVP5150_LSB_DEV_ID);
	if ( devid != 0x5150 ) {
		dev_err(&c->dev, "invalid device id 0x%04x\n", devid);
		return -EINVAL;
	}
	printk("TVP5150: ID = 0x%x \n", devid);

	/* Setup internal data */
	tvp5150_data.i2c_client      = c;
	tvp5150_data.std_id          = V4L2_STD_UNKNOWN;
	tvp5150_data.route.input     = TVP5150_COMPOSITE0;
	tvp5150_data.on              = true;

	/* Full chip reset */
	tvp5150_reset(&tvp5150_data);

	/* Register V4L2 device */
	tvp5150_int_device.priv = &tvp5150_data;
	err = v4l2_int_device_register(&tvp5150_int_device);

#ifdef DEBUG
	/* Worker to output status */
//	if ( !tvp5150_wq ) {
//		tvp5150_wq = create_singlethread_workqueue("tvp5150");
//	}
//	if ( tvp5150_wq ) {
//		queue_delayed_work(tvp5150_wq, &tvp5150_work, 1*HZ);
//	}
#endif

	return err;
}

static int tvp5150_remove(struct i2c_client *c)
{
	TRACE();

	/* Remove work queue */
#ifdef DEBUG
//	if ( tvp5150_wq ) {
//		destroy_workqueue(tvp5150_wq);
//	}
#endif

	/* Unregister */
	v4l2_int_device_unregister(&tvp5150_int_device);

	return 0;
}

/* ----------------------------------------------------------------------- */
static const struct i2c_device_id tvp5150_id[] = {
	{ "tvp5150", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tvp5150_id);

static struct i2c_driver tvp5150_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tvp5150",
	},
	.probe = tvp5150_probe,
	.remove = tvp5150_remove,
	.id_table = tvp5150_id,
};

static __init int tvp5150_init(void)
{
	u8 err = 0;

	pr_debug("In tvp5150_init\n");

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&tvp5150_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n", __func__, err);

	return err;
}

static void __exit tvp5150_clean(void)
{
	dev_dbg(&tvp5150_data.i2c_client->dev, "In tvp5150_clean\n");
	i2c_del_driver(&tvp5150_i2c_driver);
}

module_init(tvp5150_init);
module_exit(tvp5150_clean);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Anolog Device TVP5150 video decoder driver");
MODULE_LICENSE("GPL");
