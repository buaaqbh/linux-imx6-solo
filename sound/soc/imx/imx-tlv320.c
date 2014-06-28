/*
 * sound/soc/imx/3ds-tlv320.c --  SoC audio for i.MX 3ds boards with
 *                                  tlv320 codec
 *
 * Copyright 2009 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fsl_devices.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>
#include <mach/audmux.h>

#include "../codecs/tlv320aic23.h"
#include "imx-ssi.h"

static struct imx_tlv320_priv {
	int sysclk;
	int hw;
	struct platform_device *pdev;
} card_priv;

static struct snd_soc_card imx_tlv320;

static int tlv320_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	u32 dai_format;
	int ret;
	unsigned int channels = params_channels(params);

	snd_soc_dai_set_sysclk(codec_dai, 0, card_priv.sysclk, 0);

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;


	/* TODO: The SSI driver should figure this out for us */
	switch (channels) {
	case 2:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffc, 0xfffffffc, 2, 0);
		break;
	case 1:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffe, 0xfffffffe, 1, 0);
		break;
	default:
		return -EINVAL;
	}

	/* set cpu DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
		SND_SOC_DAIFMT_CBM_CFM;
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops imx_tlv320_hifi_ops = {
	.hw_params = tlv320_params,
};

/* imx_3stack card dapm widgets */
static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

/* imx_3stack machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	{"Headphone Jack", NULL, "LHPOUT"},
	{"Headphone Jack", NULL, "RHPOUT"},

	{"LLINEIN", NULL, "Line In"},
	{"RLINEIN", NULL, "Line In"},

	{"MICIN", NULL, "Mic Jack"},
};

static int imx_3stack_tlv320_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;

	/* Add imx_3stack specific widgets */
	snd_soc_dapm_new_controls(&codec->dapm, imx_3stack_dapm_widgets,
				  ARRAY_SIZE(imx_3stack_dapm_widgets));

	/* Set up imx_3stack specific audio path audio_map */
	snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(&codec->dapm, "Mic Jack");
	snd_soc_dapm_enable_pin(&codec->dapm, "Line In");
	
	snd_soc_dapm_sync(&codec->dapm);

	return 0;
}

static struct snd_soc_dai_link imx_tlv320_dai[] = {
	{
		.name		= "tlv320aic23",
		.stream_name	= "TLV320AIC23",
		.codec_dai_name	= "tlv320aic23-hifi",
		.codec_name	= "tlv320aic23-codec.0-001a",
		.cpu_dai_name	= "imx-ssi.1",
		.platform_name	= "imx-pcm-audio.1",
		.init		= imx_3stack_tlv320_init,
		.ops		= &imx_tlv320_hifi_ops,
	},
};

static struct snd_soc_card imx_tlv320 = {
	.name		= "tlv320-audio",
	.dai_link	= imx_tlv320_dai,
	.num_links	= ARRAY_SIZE(imx_tlv320_dai),
};

static struct platform_device *imx_tlv320_snd_device;

static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
	slave = slave - 1;
	master = master - 1;

	/* SSI0 mastered by port 5 */
	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(master) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = MXC_AUDMUX_V2_PTCR_SYN;
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}

static int __devinit imx_tlv320_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	int ret = 0;

	card_priv.pdev = pdev;

	imx_audmux_config(plat->src_port, plat->ext_port);

	ret = -EINVAL;
	if (plat->init && plat->init())
		return ret;

	card_priv.sysclk = plat->sysclk;

	return 0;
}

static int imx_tlv320_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->finit)
		plat->finit();

	return 0;
}

static struct platform_driver imx_tlv320_audio_driver = {
	.probe = imx_tlv320_probe,
	.remove = imx_tlv320_remove,
	.driver = {
		   .name = "imx-tlv320",
		   },
};

static int __init imx_tlv320_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_tlv320_audio_driver);
	if (ret)
		return -ENOMEM;

	if (cpu_is_mx6q())
		imx_tlv320_dai[0].codec_name = "tlv320aic23-codec.0-001a";
	else
		imx_tlv320_dai[0].codec_name = "tlv320aic23-codec.1-001a";

	imx_tlv320_snd_device = platform_device_alloc("soc-audio", 1);
	if (!imx_tlv320_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_tlv320_snd_device, &imx_tlv320);

	ret = platform_device_add(imx_tlv320_snd_device);

	if (ret) {
		printk(KERN_ERR "ASoC: Platform device allocation failed\n");
		platform_device_put(imx_tlv320_snd_device);
	}

	return ret;
}

static void __exit imx_tlv320_exit(void)
{
	platform_driver_unregister(&imx_tlv320_audio_driver);
	platform_device_unregister(imx_tlv320_snd_device);
}

module_init(imx_tlv320_init);
module_exit(imx_tlv320_exit);

MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION("PhyCORE ALSA SoC driver");
MODULE_LICENSE("GPL");
