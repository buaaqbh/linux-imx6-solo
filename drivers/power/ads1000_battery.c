#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/power_supply.h>

struct i2c_client *ads_client = NULL;

struct ads1000_device_info {
	struct device 		*dev;
	struct power_supply	bat;
	struct mutex lock;
};

static inline int ads1000_read(u8 *config, short *data)
{
	char buf[3];
	int ret;
	int power = 0;

//	printk("Enter func: %s\n", __func__);

	if (ads_client == NULL)
		return -1;

	memset(buf, 0, 3);
	ret = i2c_master_recv(ads_client, buf, 3);
	if (ret != 3) {
		printk("ADS I2C read error, ret = %d.\n", ret);
		return -1;
	}
//	printk("ADS I2C Read: 0x%x 0x%x 0x%x \n", buf[0], buf[1], buf[2]);

	*config = buf[2];
	power = (buf[0] << 8) | buf[1];
	power = (power * 330 * 127) / (27 * 2048);
	*data = (short)power;

//	printk("Power value = %d, config = 0x%x \n", power, *config);

	return 0;
}

static inline int ads1000_write_config(u8 config)
{
	int ret;

//	printk("Enter func: %s\n", __func__);

	if (ads_client == NULL)
		return -1;

	ret = i2c_master_send(ads_client, &config, 1);
	if (ret != 1) {
		printk("ADS I2C write error, ret = %d.\n", ret);
		return -1;
	}

	return 0;
}

static enum power_supply_property ads1000_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int ads1000_battery_status(struct ads1000_device_info *di,
	union power_supply_propval *val)
{
	int status;

	status = POWER_SUPPLY_STATUS_CHARGING;

	val->intval = status;

	return 0;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int ads1000_battery_voltage(struct ads1000_device_info *di,
	union power_supply_propval *val)
{
	u8 config;
	short data = 0;

	if (ads1000_read(&config, &data) < 0)
		return -1;

	val->intval = data * 10;

	return 0;
}

static int ads1000_battery_temperature(struct ads1000_device_info *di,
	union power_supply_propval *val)
{
	val->intval = 25;

	return 0;
}

static int ads1000_simple_value(int value, union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

#define to_ads1000_device_info(x) container_of((x), \
				struct ads1000_device_info, bat);
static int ads1000_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct ads1000_device_info *di = to_ads1000_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = ads1000_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = ads1000_battery_voltage(di, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = ads1000_simple_value(80, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = ads1000_battery_temperature(di, val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void ads1000_external_power_changed(struct power_supply *psy)
{
//	struct ads1000_device_info *di = to_ads1000_device_info(psy);

}

static int ads1000_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ads1000_device_info *di;
	u8 config;
	short data = 0;
	int ret;

	ads_client = client;
	ads1000_read(&config, &data);
	printk("Power value = %d \n", data);

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		ret = -ENOMEM;
		return ret;
	}

	di->dev = &client->dev;
	di->bat.name = "ADS_Battery";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = ads1000_battery_props;
	di->bat.num_properties = ARRAY_SIZE(ads1000_battery_props);
	di->bat.get_property = ads1000_battery_get_property;
	di->bat.external_power_changed = ads1000_external_power_changed;

	ret = power_supply_register(di->dev, &di->bat);
	if (ret) {
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(client, di);

	return 0;
}

static int ads1000_detach(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ads1000_id[] = {
	{"ads1000", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ads1000_id);

static struct i2c_driver ads1000_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ads1000",
		   },
	.probe = ads1000_probe,
	.remove = ads1000_detach,
	.id_table = ads1000_id,
};

static __init int ads1000_init(void)
{
	u8 err = 0;

	printk("In ads1000_init\n");

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&ads1000_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

	return err;
}

static void __exit ads1000_clean(void)
{
	i2c_del_driver(&ads1000_i2c_driver);
}

module_init(ads1000_init);
module_exit(ads1000_clean);

MODULE_AUTHOR("Qinbaohua");
MODULE_DESCRIPTION("Anolog Device ADS1000 A/D converter driver");
MODULE_LICENSE("GPL");
