#ifndef __GPIO_POWER_SABRESD_H__
#define __GPIO_POWER_SABRESD_H__

struct gpio_power_data {
	int gpio_power_12v_en;
	int gpio_power_zigbee_en;
	int gpio_power_tvp5150_en;
	int gpio_power_can_en;
	int gpio_power_rs485_en;
	int gpio_power_codec_en;
	int gpio_power_pcie_en;
	int gpio_power_wifi_en;
};

#endif
