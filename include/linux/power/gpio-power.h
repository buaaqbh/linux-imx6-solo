#ifndef __GPIO_POWER_SABRESD_H__
#define __GPIO_POWER_SABRESD_H__

struct gpio_power_data {
	int gpio_power_12v_en;
	int gpio_power_can_12v_en;
	int gpio_power_rs485_12v_en;
	int gpio_power_av_12v_en;
	int gpio_power_vout_12v_en;
	int gpio_power_zigbee_12v_en;
	int gpio_power_zigbee_en;
	int gpio_power_tvp5150_en;
	int gpio_power_can_en;
	int gpio_power_rs485_en;
	int gpio_power_codec_en;
	int gpio_power_pcie_en;
	int gpio_power_wifi_en;
	int gpio_rs485_rx_en;
	int gpio_power_nrsec_3v3;
	int gpio_power_nrsec_1v8;
	void (*rs485_enable)(void);
	void (*rs485_disable)(void);
};

#endif
