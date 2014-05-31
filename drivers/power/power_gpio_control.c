/*
 *  power_gpio_control.c
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power/gpio-power.h>
#include <linux/gpio.h>

/*
	.gpio_power_12v_en = SABRESD_12V_EN,
	.gpio_power_zigbee_en = SABRESD_ZIGBEE_PWR_EN,
	.gpio_power_tvp5150_en = SABRESD_TVP5150_PWR_EN,
	.gpio_power_can_en = SABRESD_CAN_PWR_EN,
	.gpio_power_rs485_en = SABRESD_RS485_PWR_EN,
	.gpio_power_codec_en = SABRESD_CODEC_PWR_EN,
	.gpio_power_pcie_en = SABRESD_PCIE_PWR_EN,
	.gpio_power_wifi_en = SABRESD_WIFI_PWR_EN,
	.gpio_rs485_tx_en = SABRESD_RS485_DE,
	.gpio_rs485_rx_en = SABRESD_RS485_RE,

*/

//#define P_DEBUG
#ifdef P_DEBUG
#define dprintk(format, arg...) printk(KERN_ALERT format, ## arg)
#else
#define dprintk(format, arg...) do {} while (0)
#endif

static struct gpio_power_data *pdata = NULL;
static volatile int system_12v_count = 0;
static volatile int can_12v_count = 0;
static volatile int rs485_12v_count = 0;
static volatile int zigbee_12v_count = 0;
static volatile int av_12v_count = 0;
static volatile int vout_12v_count = 0;
static volatile int zigbee_chip_count = 0;
static volatile int tvp5150_chip_count = 0;
static volatile int can_chip_count = 0;
static volatile int rs485_chip_count = 0;
static volatile int codec_chip_count = 0;
static volatile int pcie_chip_count = 0;
static volatile int wifi_chip_count = 0;

static struct mutex power_mutex;

static ssize_t power_12v_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_12v_en == -1))
		return -1;
	
	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("Power Control: System 12v Power Off, count = %d.\n", system_12v_count);
		if (system_12v_count > 0)
			system_12v_count--;
		if (system_12v_count == 0)
			gpio_set_value(pdata->gpio_power_12v_en, 0);
	}
	else if (value > 0) {
		dprintk("Power Control: System 12v Power On, count = %d.\n", system_12v_count);
		gpio_set_value(pdata->gpio_power_12v_en, 1);
		system_12v_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t power_can_12v_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_can_12v_en == -1))
		return -1;

	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("Power Control: Sensor CAN 12v Power Off, count = %d.\n", can_12v_count);
		if (can_12v_count > 0)
			can_12v_count--;
		if (can_12v_count == 0)
			gpio_set_value(pdata->gpio_power_can_12v_en, 0);
	}
	else if (value > 0) {
		dprintk("Power Control: Sensor 12v Power On, count = %d.\n", can_12v_count);
		gpio_set_value(pdata->gpio_power_12v_en, 1);
		gpio_set_value(pdata->gpio_power_can_12v_en, 1);
		can_12v_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t power_rs485_12v_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_rs485_12v_en == -1))
		return -1;

	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("\nPower Control: Sensor RS485 12v Power Off, count = %d.\n", rs485_12v_count);
		if (rs485_12v_count > 0)
			rs485_12v_count--;
		if (rs485_12v_count == 0)
			gpio_set_value(pdata->gpio_power_rs485_12v_en, 0);
	}
	else if (value == 0x5a) {
		dprintk("\nPower Control: Sensor RS485 12v Power Reset.\n");
		gpio_set_value(pdata->gpio_power_rs485_en, 1);
		gpio_set_value(pdata->gpio_power_rs485_12v_en, 0);

                gpio_set_value(pdata->gpio_rs485_rx_en, 0);
                gpio_set_value(pdata->gpio_power_rs485_en, 1);
                pdata->rs485_disable();

                msleep(800);

                pdata->rs485_enable();
                msleep(100);
                gpio_set_value(pdata->gpio_power_rs485_en, 0);

		gpio_set_value(pdata->gpio_power_12v_en, 1);
		gpio_set_value(pdata->gpio_power_rs485_12v_en, 1);
		gpio_set_value(pdata->gpio_power_rs485_en, 0);
		msleep(200);
	}
	else if (value > 0) {
		dprintk("\nPower Control: Sensor RS485 12v Power On, count = %d.\n", rs485_12v_count);
		gpio_set_value(pdata->gpio_power_12v_en, 1);
		gpio_set_value(pdata->gpio_power_rs485_12v_en, 1);
		rs485_12v_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t power_av_12v_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_av_12v_en == -1))
		return -1;

	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("Power Control: Sensor AV 12v Power Off, count = %d.\n", av_12v_count);
//		if (av_12v_count > 0)
//			av_12v_count--;
//		if (av_12v_count == 0)
		gpio_set_value(pdata->gpio_power_av_12v_en, 0);
	}
	else if (value > 0) {
		dprintk("Power Control: Sensor AV 12v Power On, count = %d.\n", av_12v_count);
		gpio_set_value(pdata->gpio_power_12v_en, 1);
		gpio_set_value(pdata->gpio_power_av_12v_en, 1);
		av_12v_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t power_vout_12v_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_vout_12v_en == -1))
		return -1;

	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("Power Control: Sensor VOUT 12v Power Off, count = %d.\n", vout_12v_count);
		if (vout_12v_count > 0)
			vout_12v_count--;
		if (vout_12v_count == 0)
			gpio_set_value(pdata->gpio_power_vout_12v_en, 0);
	}
	else if (value > 0) {
		dprintk("Power Control: Sensor VOUT 12v Power On, count = %d.\n", vout_12v_count);
		gpio_set_value(pdata->gpio_power_12v_en, 1);
		gpio_set_value(pdata->gpio_power_vout_12v_en, 1);
		vout_12v_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t power_zigbee_12v_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_zigbee_12v_en == -1))
		return -1;

	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("Power Control: Sensor Zigbee 12v Power Off, count = %d.\n", zigbee_12v_count);
		if (zigbee_12v_count > 0)
			zigbee_12v_count--;
		if (zigbee_12v_count == 0)
			gpio_set_value(pdata->gpio_power_zigbee_12v_en, 0);
	}
	else if (value > 0) {
		dprintk("Power Control: Sensor Zigbee 12v Power On, count = %d.\n", zigbee_12v_count);
		gpio_set_value(pdata->gpio_power_12v_en, 1);
		gpio_set_value(pdata->gpio_power_zigbee_12v_en, 1);
		zigbee_12v_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t power_zigbee_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_zigbee_en == -1))
		return -1;

	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("Power Control: Zigbee Power Off.\n");
		if (zigbee_chip_count > 0)
			zigbee_chip_count--;
		if (zigbee_chip_count == 0)
			gpio_set_value(pdata->gpio_power_zigbee_en, 1);
	}
	else if (value > 0) {
		dprintk("Power Control: Zigbee Power On.\n");
		gpio_set_value(pdata->gpio_power_zigbee_en, 0);
		zigbee_chip_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t power_tvp5150_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_tvp5150_en == -1))
		return -1;

	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("Power Control: TVP5150 Power Off.\n");
		if (tvp5150_chip_count > 0)
			tvp5150_chip_count--;
		if (tvp5150_chip_count == 0)
			gpio_set_value(pdata->gpio_power_tvp5150_en, 1);
	}
	else if (value > 0) {
		dprintk("Power Control: TVP5150 Power On.\n");
		gpio_set_value(pdata->gpio_power_tvp5150_en, 0);
		tvp5150_chip_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t power_can_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_can_en == -1))
		return -1;

	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("Power Control: CAN Power Off.\n");
		if (can_chip_count > 0)
			can_chip_count--;
		if (can_chip_count == 0)
			gpio_set_value(pdata->gpio_power_can_en, 0);
	}
	else if (value > 0) {
		dprintk("Power Control: CAN Power On.\n");
		gpio_set_value(pdata->gpio_power_can_en, 1);
		can_chip_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t power_rs485_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_rs485_en == -1))
		return -1;

	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("Power Control: RS485 Power Off.\n");
		if (rs485_chip_count > 0)
			rs485_chip_count--;
		if (rs485_chip_count == 0) {
			gpio_set_value(pdata->gpio_rs485_rx_en, 0);
			gpio_set_value(pdata->gpio_power_rs485_en, 1);
			pdata->rs485_disable();
		}
	}
	else if (value > 0) {
		dprintk("Power Control: RS485 Power On.\n");
		pdata->rs485_enable();
		msleep(100);
		gpio_set_value(pdata->gpio_power_rs485_en, 0);
		rs485_chip_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t power_codec_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_codec_en == -1))
		return -1;

	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("Power Control: Codec Power Off.\n");
		if (codec_chip_count > 0)
			codec_chip_count--;
		if (codec_chip_count == 0)
			gpio_set_value(pdata->gpio_power_codec_en, 1);
	}
	else if (value > 0) {
		dprintk("Power Control: Codec Power On.\n");
		gpio_set_value(pdata->gpio_power_codec_en, 0);
		codec_chip_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t power_pcie_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_pcie_en == -1))
		return -1;

	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("Power Control: PCIE Power Off.\n");
		if (pcie_chip_count > 0)
			pcie_chip_count--;
		if (pcie_chip_count == 0)
			gpio_set_value(pdata->gpio_power_pcie_en, 0);
	}
	else if (value > 0) {
		dprintk("Power Control: PCIE Power On.\n");
		gpio_set_value(pdata->gpio_power_pcie_en, 1);
		pcie_chip_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t power_wifi_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_wifi_en == -1))
		return -1;

	mutex_lock(&power_mutex);

	if (value == 0) {
		dprintk("Power Control: WIFI Power Off.\n");
		if (wifi_chip_count > 0)
			wifi_chip_count--;
		if (wifi_chip_count == 0)
			gpio_set_value(pdata->gpio_power_wifi_en, 1);
	}
	else if (value > 0) {
		dprintk("Power Control: WIFI Power On.\n");
		gpio_set_value(pdata->gpio_power_wifi_en, 0);
		wifi_chip_count++;
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	mutex_unlock(&power_mutex);

	return size;
}

static ssize_t rs485_direction_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_rs485_rx_en == -1))
		return -1;

	if (value == 0) {
//		dprintk("RS485 Direction: receive bytes.\n");
		gpio_set_value(pdata->gpio_rs485_rx_en, 0);
		mdelay(50);
	}
	else if (value > 0) {
//		dprintk("RS485 Direction: send bytes.\n");
		gpio_set_value(pdata->gpio_rs485_rx_en, 1);
		mdelay(50);
	}
	else {
		printk("RS485 Direction: Invalid Parameter.\n");
	}

	return size;
}

static DEVICE_ATTR(power_12v, 0666, NULL, power_12v_store);
static DEVICE_ATTR(power_can_12v, 0666, NULL, power_can_12v_store);
static DEVICE_ATTR(power_rs485_12v, 0666, NULL, power_rs485_12v_store);
static DEVICE_ATTR(power_av_12v, 0666, NULL, power_av_12v_store);
static DEVICE_ATTR(power_vout_12v, 0666, NULL, power_vout_12v_store);
static DEVICE_ATTR(power_zigbee_12v, 0666, NULL, power_zigbee_12v_store);
static DEVICE_ATTR(power_zigbee, 0666, NULL, power_zigbee_store);
static DEVICE_ATTR(power_tvp5150, 0666, NULL, power_tvp5150_store);
static DEVICE_ATTR(power_can, 0666, NULL, power_can_store);
static DEVICE_ATTR(power_rs485, 0666, NULL, power_rs485_store);
static DEVICE_ATTR(power_codec, 0666, NULL, power_codec_store);
static DEVICE_ATTR(power_pcie, 0666, NULL, power_pcie_store);
static DEVICE_ATTR(power_wifi, 0666, NULL, power_wifi_store);
static DEVICE_ATTR(rs485_direction, 0666, NULL, rs485_direction_store);

static int __devinit power_gpio_probe(struct platform_device *pdev)
{
	int err = 0;

	pdata = pdev->dev.platform_data;

	mutex_init(&power_mutex);

	printk(KERN_INFO "Power Control: gpio_power_12v_en = %d\n", pdata->gpio_power_12v_en);
	printk(KERN_INFO "Power Control: gpio_power_can_12v_en = %d\n", pdata->gpio_power_can_12v_en);
	printk(KERN_INFO "Power Control: gpio_power_rs485_12v_en = %d\n", pdata->gpio_power_rs485_12v_en);
	printk(KERN_INFO "Power Control: gpio_power_av_12v_en = %d\n", pdata->gpio_power_av_12v_en);
	printk(KERN_INFO "Power Control: gpio_power_vout_12v_en = %d\n", pdata->gpio_power_vout_12v_en);
	printk(KERN_INFO "Power Control: gpio_power_zigbee_12v_en = %d\n", pdata->gpio_power_zigbee_12v_en);
	printk(KERN_INFO "Power Control: gpio_power_zigbee_en = %d\n", pdata->gpio_power_zigbee_en);
	printk(KERN_INFO "Power Control: gpio_power_tvp5150_en = %d\n", pdata->gpio_power_tvp5150_en);
	printk(KERN_INFO "Power Control: gpio_power_can_en = %d\n", pdata->gpio_power_can_en);
	printk(KERN_INFO "Power Control: gpio_power_rs485_en = %d\n", pdata->gpio_power_rs485_en);
	printk(KERN_INFO "Power Control: gpio_power_codec_en = %d\n", pdata->gpio_power_codec_en);
	printk(KERN_INFO "Power Control: gpio_power_pcie_en = %d\n", pdata->gpio_power_pcie_en);
	printk(KERN_INFO "Power Control: gpio_power_wifi_en = %d\n", pdata->gpio_power_wifi_en);
	printk(KERN_INFO "RS485 Control: gpio_rs485_rx_en = %d\n", pdata->gpio_rs485_rx_en);

	err = device_create_file(&pdev->dev, &dev_attr_power_12v);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_12v.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_power_can_12v);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_can_12v.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_power_rs485_12v);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_rs485_12v.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_power_av_12v);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_av_12v.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_power_vout_12v);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_vout_12v.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_power_zigbee_12v);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_zigbee_12v.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_power_zigbee);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_zigbee.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_power_tvp5150);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_tvp5150.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_power_can);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_can.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_power_rs485);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_rs485.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_power_codec);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_codec.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_power_pcie);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_pcie.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_power_wifi);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_wifi.\n");
        }
	err = device_create_file(&pdev->dev, &dev_attr_rs485_direction);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_rs485_direction.\n");
        }

	return 0;
}

static int __devexit power_gpio_drv_remove(struct platform_device *pdev)
{
        dev_dbg(&pdev->dev, "released and freed device\n");
        return 0;
}

static struct platform_driver power_gpio_driver = {
        .driver = {
                .name    = "gpio-power",
                .owner   = THIS_MODULE,
        },
        .probe   = power_gpio_probe,
        .remove  = __devexit_p(power_gpio_drv_remove),
};

static int __init power_gpio_init(void)
{
        printk(KERN_INFO "i.MX Power GPIO Control Driver Registered.\n");

        return platform_driver_register(&power_gpio_driver);
}

static void __exit power_gpio_cleanup(void)
{
        platform_driver_unregister(&power_gpio_driver);
}

module_init(power_gpio_init);
module_exit(power_gpio_cleanup);
