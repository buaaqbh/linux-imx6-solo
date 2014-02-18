/*
 * Copyright 2005-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file tvp5150.c
 *
 * @brief Analog Device tvp5150 video decoder functions
 *
 * @ingroup Camera
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include <media/tvp5150.h>
#include "mxc_v4l2_capture.h"
#include "tvp5150_reg.h"

static struct regulator *dvddio_regulator;
static struct regulator *dvdd_regulator;
static struct regulator *avdd_regulator;
static struct regulator *pvdd_regulator;
static struct fsl_mxc_tvin_platform_data *tvin_plat;

extern void gpio_sensor_active(void);
extern void gpio_sensor_inactive(void);

static int tvp5150_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *id);
static int tvp5150_detach(struct i2c_client *client);

static const struct i2c_device_id tvp5150_id[] = {
	{"tvp5150", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tvp5150_id);

static struct i2c_driver tvp5150_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "tvp5150",
		   },
	.probe = tvp5150_probe,
	.remove = tvp5150_detach,
	.id_table = tvp5150_id,
};

/*!
 * Maintains the information on the current state of the sensor.
 */
struct sensor {
	struct sensor_data 	sen;
	struct v4l2_routing    route;
	v4l2_std_id            std_id;
	bool                   on;
} tvp5150_data;


/*! List of input video formats supported. The video formats is corresponding
 * with v4l2 id in video_fmt_t
 */
typedef enum {
	TVP5150_NTSC = 0,	/*!< Locked on (M) NTSC video signal. */
	TVP5150_PAL,		/*!< (B, G, H, I, N)PAL video signal. */
	TVP5150_NOT_LOCKED,	/*!< Not locked on a signal. */
} video_fmt_idx;

/*! Number of video standards supported (including 'not locked' signal). */
#define TVP5150_STD_MAX		(TVP5150_PAL + 1)

/*! Video format structure. */
typedef struct {
	int v4l2_id;		/*!< Video for linux ID. */
	char name[16];		/*!< Name (e.g., "NTSC", "PAL", etc.) */
	u16 raw_width;		/*!< Raw width. */
	u16 raw_height;		/*!< Raw height. */
	u16 active_width;	/*!< Active width. */
	u16 active_height;	/*!< Active height. */
} video_fmt_t;

/*! Description of video formats supported.
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
static video_fmt_t video_fmts[] = {
	{			/*! NTSC */
	 .v4l2_id = V4L2_STD_NTSC,
	 .name = "NTSC",
	 .raw_width = 720,	/* SENS_FRM_WIDTH */
	 .raw_height = 525,	/* SENS_FRM_HEIGHT */
	 .active_width = 720,	/* ACT_FRM_WIDTH plus 1 */
	 .active_height = 480,	/* ACT_FRM_WIDTH plus 1 */
	 },
	{			/*! (B, G, H, I, N) PAL */
	 .v4l2_id = V4L2_STD_PAL,
	 .name = "PAL",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 },
	{			/*! Unlocked standard */
	 .v4l2_id = V4L2_STD_ALL,
	 .name = "Autodetect",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 },
};

/*!* Standard index of tvp5150. */
static video_fmt_idx video_idx = TVP5150_PAL;

/*! @brief This mutex is used to provide mutual exclusion.
 *
 *  Create a mutex that can be used to provide mutually exclusive
 *  read/write access to the globally accessible data structures
 *  and variables that were defined above.
 */
static DEFINE_MUTEX(mutex);

#define IF_NAME                    "tvp5150"

/* supported controls */
/* This hasn't been fully implemented yet.
 * This is how it should work, though. */
static struct v4l2_queryctrl tvp5150_qctrl[] = {
	{
	.id = V4L2_CID_BRIGHTNESS,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Brightness",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 1,		/* check this value */
	.default_value = 127,	/* check this value */
	.flags = 0,
	}, {
	.id = V4L2_CID_SATURATION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Saturation",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 0x1,		/* check this value */
	.default_value = 127,	/* check this value */
	.flags = 0,
	}, {
	.id = V4L2_CID_CONTRAST,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Contrast",
	.minimum = 0,		/* check this value */
	.maximum = 255,		/* check this value */
	.step = 0x1,		/* check this value */
	.default_value = 127,	/* check this value */
	.flags = 0,
	}, {
	.id = V4L2_CID_HUE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Hue",
	.minimum = -128,
	.maximum = 127,
	.step = 0x1,
	.default_value = 0,
	.flags = 0,
	}
};

/***********************************************************************
 * I2C transfert.
 ***********************************************************************/

/* Register setting */
struct i2c_reg_value {
	u8 reg;
	u8 val;
};

/*! Read one register from a tvp5150 i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static inline int tvp5150_read(u8 reg)
{
	int val;
	val = i2c_smbus_read_byte_data(tvp5150_data.sen.i2c_client, reg);
	if (val < 0) {
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg);
		return -1;
	}
	return val;
}

/*! Write one register of a tvp5150 i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static int tvp5150_write_reg(u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(tvp5150_data.sen.i2c_client, reg, val);
	if (ret < 0) {
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return -1;
	}
	return 0;
}

/* Write array */
static int tvp5150_write_array(const struct i2c_reg_value *regs)
{
	int err = 0;
	while ( !err && (regs->reg != 0xff) ) {
		err = tvp5150_write_reg(regs->reg, regs->val);
		regs++;
	}
	return err;
}

/* **************************************************************************
 * I2C debug
 * *************************************************************************/

/* Register dump */
static void tvp5150_dump(char* pre, u8 beg, u8 end, int line)
{
	int i = 0;
	while ( beg <= end ) {
		if ( (i % line) == 0 ) {
			if ( i > 0 ) printk("\n");
			printk("tvp5150: %s reg 0x%02x = ", pre, beg);
		}
		printk("%02x ", tvp5150_read(beg));
		beg++;
		i++;
	}
	printk("\n");
}

/* Status dump */
static int tvp5150_status (int full)
{
	if ( full ) {
		printk("tvp5150: Video input source selection #1 = 0x%02x\n",
			tvp5150_read(TVP5150_VD_IN_SRC_SEL_1));
		printk("tvp5150: Analog channel controls = 0x%02x\n",
			tvp5150_read(TVP5150_ANAL_CHL_CTL));
		printk("tvp5150: Operation mode controls = 0x%02x\n",
			tvp5150_read(TVP5150_OP_MODE_CTL));
		printk("tvp5150: Miscellaneous controls = 0x%02x\n",
			tvp5150_read(TVP5150_MISC_CTL));
		printk("tvp5150: Autoswitch mask= 0x%02x\n",
			tvp5150_read(TVP5150_AUTOSW_MSK));
		printk("tvp5150: Color killer threshold control = 0x%02x\n",
			tvp5150_read(TVP5150_COLOR_KIL_THSH_CTL));
		printk("tvp5150: Luminance processing controls #1 #2 and #3 = %02x %02x %02x\n",
			tvp5150_read(TVP5150_LUMA_PROC_CTL_1),
			tvp5150_read(TVP5150_LUMA_PROC_CTL_2),
			tvp5150_read(TVP5150_LUMA_PROC_CTL_3));
		printk("tvp5150: Brightness control = 0x%02x\n",
			tvp5150_read(TVP5150_BRIGHT_CTL));
		printk("tvp5150: Color saturation control = 0x%02x\n",
			tvp5150_read(TVP5150_SATURATION_CTL));
		printk("tvp5150: Hue control = 0x%02x\n",
			tvp5150_read(TVP5150_HUE_CTL));
		printk("tvp5150: Contrast control = 0x%02x\n",
			tvp5150_read(TVP5150_CONTRAST_CTL));
		printk("tvp5150: Outputs and data rates select = 0x%02x\n",
			tvp5150_read(TVP5150_DATA_RATE_SEL));
		printk("tvp5150: Configuration shared pins = 0x%02x\n",
			tvp5150_read(TVP5150_CONF_SHARED_PIN));
		printk("tvp5150: Active video cropping start = 0x%02x%02x\n",
			tvp5150_read(TVP5150_ACT_VD_CROP_ST_MSB),
			tvp5150_read(TVP5150_ACT_VD_CROP_ST_LSB));
		printk("tvp5150: Active video cropping stop  = 0x%02x%02x\n",
			tvp5150_read(TVP5150_ACT_VD_CROP_STP_MSB),
			tvp5150_read(TVP5150_ACT_VD_CROP_STP_LSB));
		printk("tvp5150: Genlock/RTC = 0x%02x\n",
			tvp5150_read(TVP5150_GENLOCK));
		printk("tvp5150: Horizontal sync start = 0x%02x\n",
			tvp5150_read(TVP5150_HORIZ_SYNC_START));
		printk("tvp5150: Vertical blanking start = 0x%02x\n",
			tvp5150_read(TVP5150_VERT_BLANKING_START));
		printk("tvp5150: Vertical blanking stop = 0x%02x\n",
			tvp5150_read(TVP5150_VERT_BLANKING_STOP));
		printk("tvp5150: Chrominance processing control #1 and #2 = %02x %02x\n",
			tvp5150_read(TVP5150_CHROMA_PROC_CTL_1),
			tvp5150_read(TVP5150_CHROMA_PROC_CTL_2));
		printk("tvp5150: Interrupt reset register B = 0x%02x\n",
			tvp5150_read(TVP5150_INT_RESET_REG_B));
		printk("tvp5150: Interrupt enable register B = 0x%02x\n",
			tvp5150_read(TVP5150_INT_ENABLE_REG_B));
		printk("tvp5150: Interrupt configuration register B = 0x%02x\n",
			tvp5150_read(TVP5150_INTT_CONFIG_REG_B));
		printk("tvp5150: Video standard = 0x%02x\n",
			tvp5150_read(TVP5150_VIDEO_STD));
		printk("tvp5150: Chroma gain factor: Cb=0x%02x Cr=0x%02x\n",
			tvp5150_read(TVP5150_CB_GAIN_FACT),
			tvp5150_read(TVP5150_CR_GAIN_FACTOR));
		printk("tvp5150: Macrovision on counter = 0x%02x\n",
			tvp5150_read(TVP5150_MACROVISION_ON_CTR));
		printk("tvp5150: Macrovision off counter = 0x%02x\n",
			tvp5150_read(TVP5150_MACROVISION_OFF_CTR));
		printk("tvp5150: ITU-R BT.656.%d timing(TVP5150AM1 only)\n",
			(tvp5150_read(TVP5150_REV_SELECT) & 1) ? 3 : 4);
		printk("tvp5150: Device ID = %02x%02x\n",
			tvp5150_read(TVP5150_MSB_DEV_ID),
			tvp5150_read(TVP5150_LSB_DEV_ID));
		printk("tvp5150: ROM version = (hex) %02x.%02x\n",
			tvp5150_read(TVP5150_ROM_MAJOR_VER),
			tvp5150_read(TVP5150_ROM_MINOR_VER));
		printk("tvp5150: Vertical line count = 0x%02x%02x\n",
			tvp5150_read(TVP5150_VERT_LN_COUNT_MSB),
			tvp5150_read(TVP5150_VERT_LN_COUNT_LSB));
		printk("tvp5150: Interrupt status register B = 0x%02x\n",
			tvp5150_read(TVP5150_INT_STATUS_REG_B));
		printk("tvp5150: Interrupt active register B = 0x%02x\n",
			tvp5150_read(TVP5150_INT_ACTIVE_REG_B));
	} /* if ( full ) */

	printk("tvp5150: Status regs #1 to #5 = %02x %02x %02x %02x %02x\n",
		tvp5150_read(TVP5150_STATUS_REG_1),
		tvp5150_read(TVP5150_STATUS_REG_2),
		tvp5150_read(TVP5150_STATUS_REG_3),
		tvp5150_read(TVP5150_STATUS_REG_4),
		tvp5150_read(TVP5150_STATUS_REG_5));
	if ( full ) {
		tvp5150_dump("Teletext filter 1",   TVP5150_TELETEXT_FIL1_INI, TVP5150_TELETEXT_FIL1_END, 8);
		tvp5150_dump("Teletext filter 2",   TVP5150_TELETEXT_FIL2_INI, TVP5150_TELETEXT_FIL2_END, 8);

		printk("tvp5150: Teletext filter enable = 0x%02x\n",
			tvp5150_read(TVP5150_TELETEXT_FIL_ENA));
		printk("tvp5150: Interrupt status register A = 0x%02x\n",
			tvp5150_read(TVP5150_INT_STATUS_REG_A));
		printk("tvp5150: Interrupt enable register A = 0x%02x\n",
			tvp5150_read(TVP5150_INT_ENABLE_REG_A));
		printk("tvp5150: Interrupt configuration = 0x%02x\n",
			tvp5150_read(TVP5150_INT_CONF));
		printk("tvp5150: VDP status register = 0x%02x\n",
			tvp5150_read(TVP5150_VDP_STATUS_REG));
		printk("tvp5150: FIFO word count = 0x%02x\n",
			tvp5150_read(TVP5150_FIFO_WORD_COUNT));
		printk("tvp5150: FIFO interrupt threshold = 0x%02x\n",
			tvp5150_read(TVP5150_FIFO_INT_THRESHOLD));
		printk("tvp5150: FIFO reset = 0x%02x\n",
			tvp5150_read(TVP5150_FIFO_RESET));
		printk("tvp5150: Line number interrupt = 0x%02x\n",
			tvp5150_read(TVP5150_LINE_NUMBER_INT));
		printk("tvp5150: Pixel alignment register = 0x%02x%02x\n",
			tvp5150_read(TVP5150_PIX_ALIGN_REG_HIGH),
			tvp5150_read(TVP5150_PIX_ALIGN_REG_LOW));
		printk("tvp5150: FIFO output control = 0x%02x\n",
			tvp5150_read(TVP5150_FIFO_OUT_CTRL));
		printk("tvp5150: Full field enable = 0x%02x\n",
			tvp5150_read(TVP5150_FULL_FIELD_ENA));
		printk("tvp5150: Full field mode register = 0x%02x\n",
			tvp5150_read(TVP5150_FULL_FIELD_MODE_REG));
			tvp5150_dump("CC   data",   TVP5150_CC_DATA_INI, TVP5150_CC_DATA_END, 8);
		tvp5150_dump("WSS  data",   TVP5150_WSS_DATA_INI, TVP5150_WSS_DATA_END, 8);
		tvp5150_dump("VPS  data",   TVP5150_VPS_DATA_INI, TVP5150_VPS_DATA_END, 8);
		tvp5150_dump("VITC data",   TVP5150_VITC_DATA_INI, TVP5150_VITC_DATA_END, 10);
		tvp5150_dump("Line mode",   TVP5150_LINE_MODE_INI, TVP5150_LINE_MODE_END, 8);
	} /* if ( full ) */

	return 0;
}

/***********************************************************************
 * mxc_v4l2_capture interface.
 ***********************************************************************/

#if 0
/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_enable[] = {
	{
		TVP5150_MISC_CTL, 0x09
	},
	{
		0xff, 0xff
	}
};
#else
/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_enable[] = {
	{
		TVP5150_CONF_SHARED_PIN, 2
	},{	/* Automatic offset and AGC enabled */
		TVP5150_ANAL_CHL_CTL, 0x15
	},{	/* Activate YCrCb output 0x9 or 0xd ? */
		TVP5150_MISC_CTL, 0x6f
	},{	/* Activates video std autodetection for all standards */
		TVP5150_AUTOSW_MSK, 0x0
	},{	/* Default format: 0x47. For 4:2:2: 0x40 */
		TVP5150_DATA_RATE_SEL, 0x47
	},{
		TVP5150_CHROMA_PROC_CTL_1, 0x0c
	},{
		TVP5150_CHROMA_PROC_CTL_2, 0x54
	},{	/* Non documented, but initialized on WinTV USB2 */
		0x27, 0x20
	},{
		0xff,0xff
	}
};
#endif

/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_default[] = {
	{ /* 0x00 */
		TVP5150_VD_IN_SRC_SEL_1,0x00
	},
	{ /* 0x01 */
		TVP5150_ANAL_CHL_CTL,0x15
	},
	{ /* 0x02 */
		TVP5150_OP_MODE_CTL,0x00
	},
	{ /* 0x03 */
		TVP5150_MISC_CTL,0x01
	},
	{ /* 0x06 */
		TVP5150_COLOR_KIL_THSH_CTL,0x10
	},
	{ /* 0x07 */
		TVP5150_LUMA_PROC_CTL_1,0x60
	},
	{ /* 0x08 */
		TVP5150_LUMA_PROC_CTL_2,0x00
	},
	{ /* 0x09 */
		TVP5150_BRIGHT_CTL,0x80
	},
	{ /* 0x0a */
		TVP5150_SATURATION_CTL,0x80
	},
	{ /* 0x0b */
		TVP5150_HUE_CTL,0x00
	},
	{ /* 0x0c */
		TVP5150_CONTRAST_CTL,0x80
	},
	{ /* 0x0d */
		TVP5150_DATA_RATE_SEL,0x47
	},
	{ /* 0x0e */
		TVP5150_LUMA_PROC_CTL_3,0x00
	},
	{ /* 0x0f */
		TVP5150_CONF_SHARED_PIN,0x08
	},
	{ /* 0x11 */
		TVP5150_ACT_VD_CROP_ST_MSB,0x00
	},
	{ /* 0x12 */
		TVP5150_ACT_VD_CROP_ST_LSB,0x00
	},
	{ /* 0x13 */
		TVP5150_ACT_VD_CROP_STP_MSB,0x00
	},
	{ /* 0x14 */
		TVP5150_ACT_VD_CROP_STP_LSB,0x00
	},
	{ /* 0x15 */
		TVP5150_GENLOCK,0x01
	},
	{ /* 0x16 */
		TVP5150_HORIZ_SYNC_START,0x80
	},
	{ /* 0x18 */
		TVP5150_VERT_BLANKING_START,0x00
	},
	{ /* 0x19 */
		TVP5150_VERT_BLANKING_STOP,0x00
	},
	{ /* 0x1a */
		TVP5150_CHROMA_PROC_CTL_1,0x0c
	},
	{ /* 0x1b */
		TVP5150_CHROMA_PROC_CTL_2,0x14
	},
	{ /* 0x1c */
		TVP5150_INT_RESET_REG_B,0x00
	},
	{ /* 0x1d */
		TVP5150_INT_ENABLE_REG_B,0x00
	},
	{ /* 0x1e */
		TVP5150_INTT_CONFIG_REG_B,0x00
	},
	{ /* 0x28 */
		TVP5150_VIDEO_STD,0x00
	},
	{ /* 0x2e */
		TVP5150_MACROVISION_ON_CTR,0x0f
	},
	{ /* 0x2f */
		TVP5150_MACROVISION_OFF_CTR,0x01
	},
	{ /* 0xbb */
		TVP5150_TELETEXT_FIL_ENA,0x00
	},
	{ /* 0xc0 */
		TVP5150_INT_STATUS_REG_A,0x00
	},
	{ /* 0xc1 */
		TVP5150_INT_ENABLE_REG_A,0x00
	},
	{ /* 0xc2 */
		TVP5150_INT_CONF,0x04
	},
	{ /* 0xc8 */
		TVP5150_FIFO_INT_THRESHOLD,0x80
	},
	{ /* 0xc9 */
		TVP5150_FIFO_RESET,0x00
	},
	{ /* 0xca */
		TVP5150_LINE_NUMBER_INT,0x00
	},
	{ /* 0xcb */
		TVP5150_PIX_ALIGN_REG_LOW,0x4e
	},
	{ /* 0xcc */
		TVP5150_PIX_ALIGN_REG_HIGH,0x00
	},
	{ /* 0xcd */
		TVP5150_FIFO_OUT_CTRL,0x01
	},
	{ /* 0xcf */
		TVP5150_FULL_FIELD_ENA,0x00
	},
	{ /* 0xd0 */
		TVP5150_LINE_MODE_INI,0x00
	},
	{ /* 0xfc */
		TVP5150_FULL_FIELD_MODE_REG,0x7f
	},
	{ /* end of data */
		0xff,0xff
	}
};

struct tvp5150_vbi_type {
	unsigned int vbi_type;
	unsigned int ini_line;
	unsigned int end_line;
	unsigned int by_field :1;
};

struct i2c_vbi_ram_value {
	u16 reg;
	struct tvp5150_vbi_type type;
	unsigned char values[16];
};

/* This struct have the values for each supported VBI Standard
 * by
 tvp5150_vbi_types should follow the same order as vbi_ram_default
 * value 0 means rom position 0x10, value 1 means rom position 0x30
 * and so on. There are 16 possible locations from 0 to 15.
 */

static struct i2c_vbi_ram_value vbi_ram_default[] =
{
	/* FIXME: Current api doesn't handle all VBI types, those not
	   yet supported are placed under #if 0 */
#if 0
	{0x010, /* Teletext, SECAM, WST System A */
		{V4L2_SLICED_TELETEXT_SECAM,6,23,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xe7, 0x2e, 0x20, 0x26,
		  0xe6, 0xb4, 0x0e, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
#endif
	{0x030, /* Teletext, PAL, WST System B */
		{V4L2_SLICED_TELETEXT_B,6,22,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0x27, 0x2e, 0x20, 0x2b,
		  0xa6, 0x72, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
#if 0
	{0x050, /* Teletext, PAL, WST System C */
		{V4L2_SLICED_TELETEXT_PAL_C,6,22,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xe7, 0x2e, 0x20, 0x22,
		  0xa6, 0x98, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x070, /* Teletext, NTSC, WST System B */
		{V4L2_SLICED_TELETEXT_NTSC_B,10,21,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0x27, 0x2e, 0x20, 0x23,
		  0x69, 0x93, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x090, /* Tetetext, NTSC NABTS System C */
		{V4L2_SLICED_TELETEXT_NTSC_C,10,21,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xe7, 0x2e, 0x20, 0x22,
		  0x69, 0x93, 0x0d, 0x00, 0x00, 0x00, 0x15, 0x00 }
	},
	{0x0b0, /* Teletext, NTSC-J, NABTS System D */
		{V4L2_SLICED_TELETEXT_NTSC_D,10,21,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xa7, 0x2e, 0x20, 0x23,
		  0x69, 0x93, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x0d0, /* Closed Caption, PAL/SECAM */
		{V4L2_SLICED_CAPTION_625,22,22,1},
		{ 0xaa, 0x2a, 0xff, 0x3f, 0x04, 0x51, 0x6e, 0x02,
		  0xa6, 0x7b, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00 }
	},
#endif
	{0x0f0, /* Closed Caption, NTSC */
		{V4L2_SLICED_CAPTION_525,21,21,1},
		{ 0xaa, 0x2a, 0xff, 0x3f, 0x04, 0x51, 0x6e, 0x02,
		  0x69, 0x8c, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00 }
	},
	{0x110, /* Wide Screen Signal, PAL/SECAM */
		{V4L2_SLICED_WSS_625,23,23,1},
		{ 0x5b, 0x55, 0xc5, 0xff, 0x00, 0x71, 0x6e, 0x42,
		  0xa6, 0xcd, 0x0f, 0x00, 0x00, 0x00, 0x3a, 0x00 }
	},
#if 0
	{0x130, /* Wide Screen Signal, NTSC C */
		{V4L2_SLICED_WSS_525,20,20,1},
		{ 0x38, 0x00, 0x3f, 0x00, 0x00, 0x71, 0x6e, 0x43,
		  0x69, 0x7c, 0x08, 0x00, 0x00, 0x00, 0x39, 0x00 }
	},
	{0x150, /* Vertical Interval Timecode (VITC), PAL/SECAM */
		{V4l2_SLICED_VITC_625,6,22,0},
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x6d, 0x49,
		  0xa6, 0x85, 0x08, 0x00, 0x00, 0x00, 0x4c, 0x00 }
	},
	{0x170, /* Vertical Interval Timecode (VITC), NTSC */
		{V4l2_SLICED_VITC_525,10,20,0},
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x6d, 0x49,
		  0x69, 0x94, 0x08, 0x00, 0x00, 0x00, 0x4c, 0x00 }
	},
#endif
	{0x190, /* Video Program System (VPS), PAL */
		{V4L2_SLICED_VPS,16,16,0},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xba, 0xce, 0x2b, 0x0d,
		  0xa6, 0xda, 0x0b, 0x00, 0x00, 0x00, 0x60, 0x00 }
	},
	/* 0x1d0 User programmable */

	/* End of struct */
	{ (u16)-1 }
};

static int tvp5150_vdp_init(const struct i2c_vbi_ram_value *regs)
{
	unsigned int i;

	/* Disable Full Field */
	tvp5150_write_reg(TVP5150_FULL_FIELD_ENA, 0);

	/* Before programming, Line mode should be at 0xff */
	for (i=TVP5150_LINE_MODE_INI; i<=TVP5150_LINE_MODE_END; i++)
		tvp5150_write_reg(i, 0xff);

	/* Load Ram Table */
	while (regs->reg != (u16)-1 ) {
		tvp5150_write_reg(TVP5150_CONF_RAM_ADDR_HIGH,regs->reg>>8);
		tvp5150_write_reg(TVP5150_CONF_RAM_ADDR_LOW,regs->reg);

		for (i=0;i<16;i++)
			tvp5150_write_reg(TVP5150_VDP_CONF_RAM_DATA,regs->values[i]);

		regs++;
	}
	return 0;
}

/****************************************************************************
 * Basic Functions
 ****************************************************************************/

/* Select the MUX */
static inline void tvp5150_selmux ( struct sensor *s )
{
	unsigned char val;
	int opmode = 0;
	int input  = 0;

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
	pr_debug("video route in=%i, out=%i => in=%i, opmode=%i\n",
			s->route.input, s->route.output, input, opmode);
          
	/* Select */
	tvp5150_write_reg(TVP5150_OP_MODE_CTL,     opmode);
	tvp5150_write_reg(TVP5150_VD_IN_SRC_SEL_1, input);

	/* Svideo should enable YCrCb output and disable GPCL output
	 * For Composite and TV, it should be the reverse
	 */
	val = tvp5150_read(TVP5150_MISC_CTL);
	if ( s->route.input == TVP5150_SVIDEO )
		val = (val & ~0x40) | 0x10;
	else
		val = (val & ~0x10) | 0x40;

	tvp5150_write_reg(TVP5150_MISC_CTL, val);
}

static int tvp5150_set_std ( struct sensor *s, v4l2_std_id std )
{
	int fmt = 0;

//	printk("TVP5150: Enter func: %s, line: %d -------------\n", __func__, __LINE__);
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

//	printk("set video std reg to 0x%x\n", fmt);
	dev_err(&tvp5150_data.sen.i2c_client->dev, "set video std reg to %d\n", fmt);
	tvp5150_write_reg(TVP5150_VIDEO_STD, fmt);
	return 0;
}

/*!
 * Return attributes of current video standard.
 * Since this device autodetects the current standard, this function also
 * sets the values that need to be changed if the standard changes.
 * There is no set std equivalent function.
 *
 *  @return		None.
 */
static void tvp5150_get_std(v4l2_std_id *std)
{
	int tmp;
	int idx;

	dev_dbg(&tvp5150_data.sen.i2c_client->dev, "In tvp5150_get_std\n");
//	printk("TVP5150: Enter func: %s, line: %d -------------\n", __func__, __LINE__);

	/* Read the AD_RESULT to get the detect output video standard */
	tmp = (tvp5150_read(TVP5150_STATUS_REG_5) > 1) & 0x07;

	mutex_lock(&mutex);
	if (tmp == 0x01) {
		/* PAL */
		*std = V4L2_STD_PAL;
		idx = TVP5150_PAL;
	} else if (tmp == 0x00) {
		/*NTSC*/
		*std = V4L2_STD_NTSC;
		idx = TVP5150_NTSC;
	} else {
		*std = V4L2_STD_ALL;
		idx = TVP5150_NOT_LOCKED;
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"Got invalid video standard!\n");
	}
	mutex_unlock(&mutex);

	/* This assumes autodetect which this device uses. */
	if (*std != tvp5150_data.std_id) {
		video_idx = idx;
		tvp5150_data.std_id = *std;
		tvp5150_data.sen.pix.width = video_fmts[video_idx].raw_width;
		tvp5150_data.sen.pix.height = video_fmts[video_idx].raw_height;
	}
}


/* Reset the chip */
static int tvp5150_reset (struct sensor *s)
{
	u8 msb_id, lsb_id, msb_rom, lsb_rom;

	msb_id=tvp5150_read(TVP5150_MSB_DEV_ID);
	lsb_id=tvp5150_read(TVP5150_LSB_DEV_ID);
	msb_rom=tvp5150_read(TVP5150_ROM_MAJOR_VER);
	lsb_rom=tvp5150_read(TVP5150_ROM_MINOR_VER);

	dev_err(&tvp5150_data.sen.i2c_client->dev, "rom version %d.%d\n", msb_rom, lsb_rom);

	/* TVP5150AM1 */
	if ( msb_rom == 4 && lsb_rom == 0 ) {
		dev_err(&tvp5150_data.sen.i2c_client->dev, "tvp5150am1 detected\n");
		tvp5150_write_reg(TVP5150_REV_SELECT, 0);
	/* TVP5150A */
	} else {
		dev_err(&tvp5150_data.sen.i2c_client->dev, "tvp5150a detected\n");
	}

	/* Initializes TVP5150 to its default values */
	tvp5150_write_array(tvp5150_init_default);

	/* Initializes VDP registers */
	tvp5150_vdp_init(vbi_ram_default);

	/* Initialiss TVP5150 to stream enabled values */
	tvp5150_write_array(tvp5150_init_enable);

	/* Select input */
	tvp5150_selmux(s);

	/* Initialize image preferences */
	tvp5150_write_reg(TVP5150_BRIGHT_CTL, 128);
	tvp5150_write_reg(TVP5150_CONTRAST_CTL, 128);
	tvp5150_write_reg(TVP5150_SATURATION_CTL, 128);
	tvp5150_write_reg(TVP5150_HUE_CTL, 0);

	/* Ready */
	tvp5150_set_std(s, s->std_id);

//	tvp5150_status(1);

	return 0;
}

/***********************************************************************
 * IOCTL Functions from v4l2_int_ioctl_desc.
 ***********************************************************************/

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	dev_dbg(&tvp5150_data.sen.i2c_client->dev, "tvp5150:ioctl_g_ifparm\n");
//	printk("TVP5150: Enter func: %s, line: %d -------------\n", __func__, __LINE__);

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
//	p->u.bt656.nobt_hs_inv = 1;
	p->u.bt656.bt_sync_correct = 1;

	/* tvp5150 has a dedicated clock so no clock settings needed. */

	return 0;
}

/*!
 * Sets the camera power.
 *
 * s  pointer to the camera device
 * on if 1, power is to be turned on.  0 means power is to be turned off
 *
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 * This is called on open, close, suspend and resume.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor *sensor = s->priv;

	dev_dbg(&tvp5150_data.sen.i2c_client->dev, "tvp5150:ioctl_s_power\n");
//	printk("TVP5150: Enter func: %s, line: %d, on = %d -------------\n", __func__, __LINE__, on);

	if (on && !sensor->sen.on) {
		gpio_sensor_active();
//		if (tvp5150_write_reg(TVP5150_OP_MODE_CTL, 0x00) != 0)
//			return -EIO;

		/* Full chip reset */
		tvp5150_reset(&tvp5150_data);

		/*
		 * FIXME:Additional 400ms to wait the chip to be stable?
		 * This is a workaround for preview scrolling issue.
		 */
		msleep(400);
	} else if (!on && sensor->sen.on) {
//		if (tvp5150_write_reg(TVP5150_OP_MODE_CTL, 0x01) != 0)
//			return -EIO;
		gpio_sensor_inactive();
	}

	sensor->sen.on = on;

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	dev_dbg(&tvp5150_data.sen.i2c_client->dev, "In tvp5150:ioctl_g_parm\n");
//	printk("TVP5150: Enter func: %s, line: %d -------------\n", __func__, __LINE__);

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->sen.streamcap.capability;
		cparm->timeperframe = sensor->sen.streamcap.timeperframe;
		cparm->capturemode = sensor->sen.streamcap.capturemode;
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 *
 * This driver cannot change these settings.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	dev_dbg(&tvp5150_data.sen.i2c_client->dev, "In tvp5150:ioctl_s_parm\n");
//	printk("TVP5150: Enter func: %s, line: %d -------------\n", __func__, __LINE__);

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor *sensor = s->priv;

	dev_dbg(&tvp5150_data.sen.i2c_client->dev, "tvp5150:ioctl_g_fmt_cap\n");
//	printk("TVP5150: Enter func: %s, line: %d -------------\n", __func__, __LINE__);

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   Returning size of %dx%d\n",
			 sensor->sen.pix.width, sensor->sen.pix.height);
		f->fmt.pix = sensor->sen.pix;
		break;

	case V4L2_BUF_TYPE_PRIVATE: {
		v4l2_std_id std;
		tvp5150_get_std(&std);
		f->fmt.pix.pixelformat = (u32)std;
		}
		break;

	default:
		f->fmt.pix = sensor->sen.pix;
		break;
	}

	return 0;
}

/*!
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
			   struct v4l2_queryctrl *qc)
{
	int i;

	dev_dbg(&tvp5150_data.sen.i2c_client->dev, "tvp5150:ioctl_queryctrl\n");
//	printk("TVP5150: Enter func: %s, line: %d -------------\n", __func__, __LINE__);

	for (i = 0; i < ARRAY_SIZE(tvp5150_qctrl); i++)
		if (qc->id && qc->id == tvp5150_qctrl[i].id) {
			memcpy(qc, &(tvp5150_qctrl[i]),
				sizeof(*qc));
			return 0;
		}

	return -EINVAL;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;
	int sat = 0;

	dev_dbg(&tvp5150_data.sen.i2c_client->dev, "In tvp5150:ioctl_g_ctrl\n");
//	printk("TVP5150: Enter func: %s, line: %d -------------\n", __func__, __LINE__);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		tvp5150_data.sen.brightness = tvp5150_read(TVP5150_BRIGHT_CTL);
		vc->value = tvp5150_data.sen.brightness;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		tvp5150_data.sen.contrast = tvp5150_read(TVP5150_CONTRAST_CTL);
		vc->value = tvp5150_data.sen.contrast;
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		sat = tvp5150_read(TVP5150_SATURATION_CTL);
		tvp5150_data.sen.saturation = sat;
		vc->value = tvp5150_data.sen.saturation;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_HUE\n");
		tvp5150_data.sen.hue = tvp5150_read(TVP5150_HUE_CTL);
		vc->value = tvp5150_data.sen.hue;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
		vc->value = tvp5150_data.sen.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		vc->value = tvp5150_data.sen.blue;
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		vc->value = tvp5150_data.sen.ae_mode;
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   Default case\n");
		vc->value = 0;
		ret = -EPERM;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	u8 tmp;

	dev_dbg(&tvp5150_data.sen.i2c_client->dev, "In tvp5150:ioctl_s_ctrl\n");
//	printk("TVP5150: Enter func: %s, line: %d -------------\n", __func__, __LINE__);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		tmp = vc->value;
		tvp5150_write_reg(TVP5150_BRIGHT_CTL, tmp);
		tvp5150_data.sen.brightness = vc->value;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		tmp = vc->value;
		tvp5150_write_reg(TVP5150_CONTRAST_CTL, tmp);
		tvp5150_data.sen.contrast = vc->value;
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		tmp = vc->value;
		tvp5150_write_reg(TVP5150_SATURATION_CTL, tmp);
		tvp5150_data.sen.saturation = vc->value;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_HUE\n");
		tmp = vc->value;
		tvp5150_write_reg(TVP5150_HUE_CTL, tmp);
		tvp5150_data.sen.hue = vc->value;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&tvp5150_data.sen.i2c_client->dev,
			"   Default case\n");
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index >= 1)
		return -EINVAL;

	fsize->discrete.width = video_fmts[video_idx].active_width;
	fsize->discrete.height  = video_fmts[video_idx].active_height;

	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
						"tvp5150_decoder");
	((struct v4l2_dbg_chip_ident *)id)->ident = V4L2_IDENT_TVP5150;

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	dev_dbg(&tvp5150_data.sen.i2c_client->dev, "In tvp5150:ioctl_init\n");
	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	dev_dbg(&tvp5150_data.sen.i2c_client->dev, "tvp5150:ioctl_dev_init\n");
	return 0;
}

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc tvp5150_ioctl_desc[] = {

	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*)ioctl_dev_init},

	/*!
	 * Delinitialise the dev. at slave detach.
	 * The complement of ioctl_dev_init.
	 */
/*	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func *)ioctl_dev_exit}, */

	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func*)ioctl_init},

	/*!
	 * VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
	 */
/*	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap}, */

	/*!
	 * VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.
	 * This ioctl is used to negotiate the image capture size and
	 * pixel format without actually making it take effect.
	 */
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */

	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*)ioctl_g_fmt_cap},

	/*!
	 * If the requested format is supported, configures the HW to use that
	 * format, returns error code if format not supported or HW can't be
	 * correctly configured.
	 */
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */

	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func*)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func*)ioctl_s_parm},
	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func*)ioctl_queryctrl},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func*)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func*)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave tvp5150_slave = {
	.ioctls = tvp5150_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(tvp5150_ioctl_desc),
};

static struct v4l2_int_device tvp5150_int_device = {
	.module = THIS_MODULE,
	.name = "tvp5150",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &tvp5150_slave,
	},
};


/***********************************************************************
 * I2C client and driver.
 ***********************************************************************/

/*! tvp5150 Reset function.
 *
 *  @return		None.
 */
static void tvp5150_hard_reset(bool cvbs)
{
	dev_dbg(&tvp5150_data.sen.i2c_client->dev,
		"In tvp5150:tvp5150_hard_reset\n");

	if (cvbs) {
		/* Set CVBS input on AIN1 */
		tvp5150_write_reg(TVP5150_VD_IN_SRC_SEL_1, 0x00);
	} else {
		/*
		 * Set YPbPr input on AIN1,4,5 and normal
		 * operations(autodection of all stds).
		 */
		tvp5150_write_reg(TVP5150_VD_IN_SRC_SEL_1, 0x09);
	}

	/* Datasheet recommends */
}

/*! tvp5150 I2C attach function.
 *
 *  @param *adapter	struct i2c_adapter *.
 *
 *  @return		Error code indicating success or failure.
 */

/*!
 * tvp5150 I2C probe function.
 * Function set in i2c_driver struct.
 * Called by insmod.
 *
 *  @param *adapter	I2C adapter descriptor.
 *
 *  @return		Error code indicating success or failure.
 */
static int tvp5150_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int rev_id;
	int ret = 0;
	tvin_plat = client->dev.platform_data;

	pr_debug("In tvp5150_probe\n");

	if (tvin_plat->dvddio_reg) {
		dvddio_regulator =
		    regulator_get(&client->dev, tvin_plat->dvddio_reg);
		if (!IS_ERR_VALUE((unsigned long)dvddio_regulator)) {
			regulator_set_voltage(dvddio_regulator, 3300000, 3300000);
			if (regulator_enable(dvddio_regulator) != 0)
				return -ENODEV;
		}
	}

	if (tvin_plat->dvdd_reg) {
		dvdd_regulator =
		    regulator_get(&client->dev, tvin_plat->dvdd_reg);
		if (!IS_ERR_VALUE((unsigned long)dvdd_regulator)) {
			regulator_set_voltage(dvdd_regulator, 1800000, 1800000);
			if (regulator_enable(dvdd_regulator) != 0)
				return -ENODEV;
		}
	}

	if (tvin_plat->avdd_reg) {
		avdd_regulator =
		    regulator_get(&client->dev, tvin_plat->avdd_reg);
		if (!IS_ERR_VALUE((unsigned long)avdd_regulator)) {
			regulator_set_voltage(avdd_regulator, 1800000, 1800000);
			if (regulator_enable(avdd_regulator) != 0)
				return -ENODEV;
		}
	}

	if (tvin_plat->pvdd_reg) {
		pvdd_regulator =
		    regulator_get(&client->dev, tvin_plat->pvdd_reg);
		if (!IS_ERR_VALUE((unsigned long)pvdd_regulator)) {
			regulator_set_voltage(pvdd_regulator, 1800000, 1800000);
			if (regulator_enable(pvdd_regulator) != 0)
				return -ENODEV;
		}
	}

	if (tvin_plat->io_init)
		tvin_plat->io_init();

	if (tvin_plat->reset)
		tvin_plat->reset();

	if (tvin_plat->pwdn)
		tvin_plat->pwdn(0);

	msleep(1);

	/* Set initial values for the sensor struct. */
	memset(&tvp5150_data, 0, sizeof(tvp5150_data));
	tvp5150_data.sen.i2c_client = client;
	tvp5150_data.sen.streamcap.timeperframe.denominator = 30;
	tvp5150_data.sen.streamcap.timeperframe.numerator = 1;
	tvp5150_data.std_id = V4L2_STD_ALL;
	video_idx = TVP5150_NOT_LOCKED;
	tvp5150_data.sen.pix.width = video_fmts[video_idx].raw_width;
	tvp5150_data.sen.pix.height = video_fmts[video_idx].raw_height;
	tvp5150_data.sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;  /* YUV422 */
	tvp5150_data.sen.pix.priv = 1;  /* 1 is used to indicate TV in */
	tvp5150_data.sen.on = true;
	tvp5150_data.route.input     = TVP5150_COMPOSITE0;
	tvp5150_data.on              = true;

	gpio_sensor_active();

	dev_dbg(&tvp5150_data.sen.i2c_client->dev,
		"%s:tvp5150 probe i2c address is 0x%02X\n",
		__func__, tvp5150_data.sen.i2c_client->addr);

	/*! Read the revision ID of the tvin chip */
	rev_id = tvp5150_read(TVP5150_MSB_DEV_ID) << 8;
	rev_id |= tvp5150_read(TVP5150_LSB_DEV_ID);
//	printk("--------- id = 0x%x ---------\n", rev_id);
	dev_err(&tvp5150_data.sen.i2c_client->dev,
		"%s:Analog Device tvp%2X0 detected!\n", __func__,
		rev_id);

	/*! tvp5150 initialization. */
	tvp5150_hard_reset(tvin_plat->cvbs);

	/* Full chip reset */
	tvp5150_reset(&tvp5150_data);

	pr_debug("   type is %d (expect %d)\n",
		 tvp5150_int_device.type, v4l2_int_type_slave);
	pr_debug("   num ioctls is %d\n",
		 tvp5150_int_device.u.slave->num_ioctls);

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the mt9v111_data structure here.*/
	tvp5150_int_device.priv = &tvp5150_data;
	ret = v4l2_int_device_register(&tvp5150_int_device);

	return ret;
}

/*!
 * tvp5150 I2C detach function.
 * Called on rmmod.
 *
 *  @param *client	struct i2c_client*.
 *
 *  @return		Error code indicating success or failure.
 */
static int tvp5150_detach(struct i2c_client *client)
{
	dev_dbg(&tvp5150_data.sen.i2c_client->dev,
		"%s:Removing %s video decoder @ 0x%02X from adapter %s\n",
		__func__, IF_NAME, client->addr << 1, client->adapter->name);

	/* Power down via i2c */
	tvp5150_write_reg(TVP5150_OP_MODE_CTL, 0x01);

	if (dvddio_regulator) {
		regulator_disable(dvddio_regulator);
		regulator_put(dvddio_regulator);
	}

	if (dvdd_regulator) {
		regulator_disable(dvdd_regulator);
		regulator_put(dvdd_regulator);
	}

	if (avdd_regulator) {
		regulator_disable(avdd_regulator);
		regulator_put(avdd_regulator);
	}

	if (pvdd_regulator) {
		regulator_disable(pvdd_regulator);
		regulator_put(pvdd_regulator);
	}

	v4l2_int_device_unregister(&tvp5150_int_device);

	return 0;
}

/*!
 * tvp5150 init function.
 * Called on insmod.
 *
 * @return    Error code indicating success or failure.
 */
static __init int tvp5150_init(void)
{
	u8 err = 0;

	pr_debug("In tvp5150_init\n");

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&tvp5150_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

	return err;
}

/*!
 * tvp5150 cleanup function.
 * Called on rmmod.
 *
 * @return   Error code indicating success or failure.
 */
static void __exit tvp5150_clean(void)
{
	dev_dbg(&tvp5150_data.sen.i2c_client->dev, "In tvp5150_clean\n");
	i2c_del_driver(&tvp5150_i2c_driver);
	gpio_sensor_inactive();
}

module_init(tvp5150_init);
module_exit(tvp5150_clean);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Anolog Device tvp5150 video decoder driver");
MODULE_LICENSE("GPL");
