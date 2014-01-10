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

*/
static struct gpio_power_data *pdata = NULL;

static ssize_t power_12v_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_12v_en == -1))
		return -1;

	if (value == 0) {
		printk("Power Control: 12V Power Off.\n");
		gpio_set_value(pdata->gpio_power_zigbee_en, 1);
	}
	else if (value > 0) {
		printk("Power Control: 12V Power On.\n");
		gpio_set_value(pdata->gpio_power_12v_en, 0);
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	return size;
}

static ssize_t power_zigbee_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_zigbee_en == -1))
		return -1;

	if (value == 0) {
		printk("Power Control: Zigbee Power Off.\n");
		gpio_set_value(pdata->gpio_power_zigbee_en, 1);
	}
	else if (value > 0) {
		printk("Power Control: Zigbee Power On.\n");
		gpio_set_value(pdata->gpio_power_zigbee_en, 0);
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	return size;
}

static ssize_t power_tvp5150_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_tvp5150_en == -1))
		return -1;

	if (value == 0) {
		printk("Power Control: TVP5150 Power Off.\n");
		gpio_set_value(pdata->gpio_power_tvp5150_en, 1);
	}
	else if (value > 0) {
		printk("Power Control: TVP5150 Power On.\n");
		gpio_set_value(pdata->gpio_power_tvp5150_en, 0);
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	return size;
}

static ssize_t power_can_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_can_en == -1))
		return -1;

	if (value == 0) {
		printk("Power Control: CAN Power Off.\n");
		gpio_set_value(pdata->gpio_power_can_en, 0);
	}
	else if (value > 0) {
		printk("Power Control: CAN Power On.\n");
		gpio_set_value(pdata->gpio_power_can_en, 1);
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	return size;
}

static ssize_t power_rs485_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_rs485_en == -1))
		return -1;

	if (value == 0) {
		printk("Power Control: RS485 Power Off.\n");
		gpio_set_value(pdata->gpio_power_rs485_en, 1);
	}
	else if (value > 0) {
		printk("Power Control: RS485 Power On.\n");
		gpio_set_value(pdata->gpio_power_rs485_en, 0);
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	return size;
}

static ssize_t power_codec_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_codec_en == -1))
		return -1;

	if (value == 0) {
		printk("Power Control: Codec Power Off.\n");
		gpio_set_value(pdata->gpio_power_codec_en, 1);
	}
	else if (value > 0) {
		printk("Power Control: Codec Power On.\n");
		gpio_set_value(pdata->gpio_power_codec_en, 0);
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	return size;
}

static ssize_t power_pcie_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_pcie_en == -1))
		return -1;

	if (value == 0) {
		printk("Power Control: PCIE Power Off.\n");
		gpio_set_value(pdata->gpio_power_pcie_en, 1);
	}
	else if (value > 0) {
		printk("Power Control: PCIE Power On.\n");
		gpio_set_value(pdata->gpio_power_pcie_en, 0);
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	return size;
}

static ssize_t power_wifi_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	size_t status;
	long value;

	status = strict_strtol(buf, 0, &value);

	if ((pdata == NULL) || (pdata->gpio_power_wifi_en == -1))
		return -1;

	if (value == 0) {
		printk("Power Control: WIFI Power Off.\n");
		gpio_set_value(pdata->gpio_power_wifi_en, 1);
	}
	else if (value > 0) {
		printk("Power Control: WIFI Power On.\n");
		gpio_set_value(pdata->gpio_power_wifi_en, 0);
	}
	else {
		printk("Power Control: Invalid Parameter.\n");
	}

	return size;
}

static DEVICE_ATTR(power_12v, 0666, NULL, power_12v_store);
static DEVICE_ATTR(power_zigbee, 0666, NULL, power_zigbee_store);
static DEVICE_ATTR(power_tvp5150, 0666, NULL, power_tvp5150_store);
static DEVICE_ATTR(power_can, 0666, NULL, power_can_store);
static DEVICE_ATTR(power_rs485, 0666, NULL, power_rs485_store);
static DEVICE_ATTR(power_codec, 0666, NULL, power_codec_store);
static DEVICE_ATTR(power_pcie, 0666, NULL, power_pcie_store);
static DEVICE_ATTR(power_wifi, 0666, NULL, power_wifi_store);

static int __devinit power_gpio_probe(struct platform_device *pdev)
{
	int err = 0;

	pdata = pdev->dev.platform_data;

	printk(KERN_INFO "Power Control: gpio_power_12v_en = %d\n", pdata->gpio_power_12v_en);
	printk(KERN_INFO "Power Control: gpio_power_zigbee_en = %d\n", pdata->gpio_power_zigbee_en);
	printk(KERN_INFO "Power Control: gpio_power_tvp5150_en = %d\n", pdata->gpio_power_tvp5150_en);
	printk(KERN_INFO "Power Control: gpio_power_can_en = %d\n", pdata->gpio_power_can_en);
	printk(KERN_INFO "Power Control: gpio_power_rs485_en = %d\n", pdata->gpio_power_rs485_en);
	printk(KERN_INFO "Power Control: gpio_power_codec_en = %d\n", pdata->gpio_power_codec_en);
	printk(KERN_INFO "Power Control: gpio_power_pcie_en = %d\n", pdata->gpio_power_pcie_en);
	printk(KERN_INFO "Power Control: gpio_power_wifi_en = %d\n", pdata->gpio_power_wifi_en);

	err = device_create_file(&pdev->dev, &dev_attr_power_12v);
        if (err != 0) {
                printk(KERN_ERR "Power Control: cannot create FILE dev_attr_power_12v.\n");
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
