/*
 * ASoC driver for TI DAVINCI EVM platform
 *
 * Author:      Vladimir Barinov, <vbarinov@embeddedalley.com>
 * Copyright:   (C) 2007 MontaVista Software, Inc., <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/platform_data/edma.h>
#include <linux/i2c.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <asm/dma.h>
#include <asm/mach-types.h>

#include <linux/edma.h>

#include "davinci-pcm.h"
#include "davinci-i2s.h"

struct snd_soc_card_drvdata_davinci {
	struct clk *mclk;
	unsigned sysclk;
	unsigned clk_gpio;
	struct snd_pcm_hw_constraint_list *rate_constraint;
};

/* If changing sample format the tda998x configuration (REG_CTS_N) needs
   to be changed. */
#define TDA998X_SAMPLE_FORMAT SNDRV_PCM_FORMAT_S32_LE

static unsigned int evm_get_bclk(struct snd_pcm_hw_params *params)
{
	int sample_size = snd_pcm_format_width(params_format(params));
	int rate = params_rate(params);
	int channels = params_channels(params);

	return sample_size * channels * rate;
}

static int evm_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->codec->card;
	struct clk *mclk = ((struct snd_soc_card_drvdata_davinci *)
			    snd_soc_card_get_drvdata(soc_card))->mclk;
	if (mclk)
		return clk_prepare_enable(mclk);

	return 0;
}

static int dra7xx_startup(struct snd_pcm_substream *substream)
{
	snd_pcm_hw_constraint_minmax(substream->runtime,
				     SNDRV_PCM_HW_PARAM_RATE, 44100, 44100);

	return evm_startup(substream);
}

static void evm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->codec->card;
	struct clk *mclk = ((struct snd_soc_card_drvdata_davinci *)
			    snd_soc_card_get_drvdata(soc_card))->mclk;
	if (mclk)
		clk_disable_unprepare(mclk);
}

static int evm_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *soc_card = codec->card;
	int ret = 0;
	unsigned sysclk = ((struct snd_soc_card_drvdata_davinci *)
			   snd_soc_card_get_drvdata(soc_card))->sysclk;
	/* set the codec system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, sysclk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* set the CPU system clock */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;
	
	return 0;
}

static int evm_tda998x_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->codec->card;
	struct snd_soc_card_drvdata_davinci *drvdata =
		(struct snd_soc_card_drvdata_davinci *)
		snd_soc_card_get_drvdata(soc_card);
	struct snd_mask *fmt = constrs_mask(&runtime->hw_constraints,
					    SNDRV_PCM_HW_PARAM_FORMAT);
	snd_mask_none(fmt);
	snd_mask_set(fmt, TDA998X_SAMPLE_FORMAT);

	runtime->hw.rate_min = drvdata->rate_constraint->list[0];
	runtime->hw.rate_max = drvdata->rate_constraint->list[
		drvdata->rate_constraint->count - 1];
	runtime->hw.rates = SNDRV_PCM_RATE_KNOT;

	snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
				   drvdata->rate_constraint);
	snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_CHANNELS,
				     2, 2);

	return evm_startup(substream);
}

static int evm_tda998x_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *soc_card = codec->card;
	struct platform_device *pdev = to_platform_device(soc_card->dev);
	unsigned int bclk_freq = evm_get_bclk(params);
	unsigned sysclk = ((struct snd_soc_card_drvdata_davinci *)
			   snd_soc_card_get_drvdata(soc_card))->sysclk;
	int ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, 1, sysclk / bclk_freq);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't set CPU DAI clock divider %d\n",
			ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return ret;
}

static int evm_slave_codec_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *soc_card = codec->card;
	unsigned int bclk_freq = evm_get_bclk(params);
	int ret = 0;
	unsigned sysclk = ((struct snd_soc_card_drvdata_davinci *)
			   snd_soc_card_get_drvdata(soc_card))->sysclk;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, 1, sysclk / bclk_freq);
	if (ret < 0) {
		dev_err(soc_card->dev, "can't set CPU DAI clock divider %d\n",
			ret);
	}

	/* Set MCLK as clock source for tlv320aic3106 */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, sysclk, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* Set McASP sysclk from AHCLKX sourced from ATL */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk, SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(soc_card->dev, "can't set CPU DAI sysclk %d\n", ret);

	return ret;
}

static struct snd_soc_ops evm_ops = {
	.startup = evm_startup,
	.shutdown = evm_shutdown,
	.hw_params = evm_hw_params,
};

static struct snd_soc_ops evm_spdif_ops = {
	.startup = evm_startup,
	.shutdown = evm_shutdown,
};

static struct snd_soc_ops evm_tda998x_ops = {
	.startup = evm_tda998x_startup,
	.shutdown = evm_shutdown,
	.hw_params = evm_tda998x_hw_params,
};

static struct snd_soc_ops dra7xx_ops = {
	.startup = dra7xx_startup,
	.shutdown = evm_shutdown,
	.hw_params = evm_slave_codec_hw_params,
};

/* davinci-evm machine dapm widgets */
static const struct snd_soc_dapm_widget aic3x_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

/* davinci-evm machine audio_mapnections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Headphone connected to HPLOUT, HPROUT */
	{"Headphone Jack", NULL, "HPLOUT"},
	{"Headphone Jack", NULL, "HPROUT"},

	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "LLOUT"},
	{"Line Out", NULL, "RLOUT"},

	/* Mic connected to (MIC3L | MIC3R) */
	{"MIC3L", NULL, "Mic Bias"},
	{"MIC3R", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic Jack"},

	/* Line In connected to (LINE1L | LINE2L), (LINE1R | LINE2R) */
	{"LINE1L", NULL, "Line In"},
	{"LINE2L", NULL, "Line In"},
	{"LINE1R", NULL, "Line In"},
	{"LINE2R", NULL, "Line In"},
};

/* Logic for a aic3x as connected on a davinci-evm */
static int evm_aic3x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct device_node *np = codec->card->dev->of_node;
	int ret;

	/* Add davinci-evm specific widgets */
	snd_soc_dapm_new_controls(dapm, aic3x_dapm_widgets,
				  ARRAY_SIZE(aic3x_dapm_widgets));

	if (np) {
		ret = snd_soc_of_parse_audio_routing(codec->card,
							"ti,audio-routing");
		if (ret)
			return ret;
	} else {
		/* Set up davinci-evm specific audio path audio_map */
		snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));
	}

	/* not connected */
	/*snd_soc_dapm_disable_pin(dapm, "MONO_LOUT");
	snd_soc_dapm_disable_pin(dapm, "HPLCOM");
	snd_soc_dapm_disable_pin(dapm, "HPRCOM");*/

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(dapm, "Line Out");
	snd_soc_dapm_enable_pin(dapm, "Mic Jack");
	snd_soc_dapm_enable_pin(dapm, "Line In");

	return 0;
}

static unsigned int tda998x_hdmi_rates[] = {
	32000,
	44100,
	48000,
	88200,
	96000,
};

static struct snd_pcm_hw_constraint_list *evm_tda998x_rate_constraint(
	struct snd_soc_card *soc_card)
{
	struct platform_device *pdev = to_platform_device(soc_card->dev);
	unsigned sysclk = ((struct snd_soc_card_drvdata_davinci *)
			   snd_soc_card_get_drvdata(soc_card))->sysclk;
	struct snd_pcm_hw_constraint_list *ret;
	unsigned int *rates;
	int i = 0, j = 0;

	ret = devm_kzalloc(soc_card->dev, sizeof(*ret) +
			   sizeof(tda998x_hdmi_rates), GFP_KERNEL);
	if (!ret) {
		dev_err(&pdev->dev, "Unable to allocate rate constraint!\n");
		return NULL;
	}

	ret->list = rates = (unsigned int *) &ret[1];
	ret->mask = 0;
	for (; i < ARRAY_SIZE(tda998x_hdmi_rates); i++) {
		unsigned int bclk_freq = tda998x_hdmi_rates[i] * 2 *
			snd_pcm_format_width(TDA998X_SAMPLE_FORMAT);
		if (sysclk % bclk_freq == 0) {
			rates[j++] = tda998x_hdmi_rates[i];
			dev_dbg(soc_card->dev, "Allowing rate %u\n",
				tda998x_hdmi_rates[i]);
		}
	}
	ret->count = j;
	return ret;
}

static const struct snd_soc_dapm_widget tda998x_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("HDMI Out"),
};

static int evm_tda998x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dapm_context *dapm = &rtd->codec->dapm;
	struct snd_soc_card *soc_card = rtd->codec->card;
	struct snd_soc_card_drvdata_davinci *drvdata =
		(struct snd_soc_card_drvdata_davinci *)
		snd_soc_card_get_drvdata(soc_card);
	int ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, 0, 1);
	if (ret < 0)
		return ret;

	drvdata->rate_constraint = evm_tda998x_rate_constraint(soc_card);

	snd_soc_dapm_new_controls(dapm, tda998x_dapm_widgets,
				  ARRAY_SIZE(tda998x_dapm_widgets));

	ret = snd_soc_of_parse_audio_routing(soc_card, "ti,audio-routing");

	/* not connected */
	snd_soc_dapm_disable_pin(dapm, "RX");

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "HDMI Out");

	return 0;
}

static const struct snd_soc_dapm_widget aic31xx_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static const struct snd_kcontrol_new aic31xx_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
};

/* Logic for EVMs with an aic31xx */
static int evm_aic31xx_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct device_node *np = codec->card->dev->of_node;
	int ret;

	snd_soc_dapm_new_controls(dapm, aic31xx_dapm_widgets,
				  ARRAY_SIZE(aic31xx_dapm_widgets));

	if (np) {
		ret = snd_soc_of_parse_audio_routing(codec->card,
						     "ti,audio-routing");
		if (ret)
			return ret;
	}

	/* not connected */
	snd_soc_dapm_disable_pin(dapm, "MIC1LM");

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(dapm, "Speaker");
	snd_soc_dapm_enable_pin(dapm, "Mic Jack");

	return 0;
}

/* davinci-evm digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link dm6446_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcbsp",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-001b",
	.platform_name = "davinci-mcbsp",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
	.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM |
		   SND_SOC_DAIFMT_IB_NF,
};

static struct snd_soc_dai_link dm355_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcbsp.1",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-001b",
	.platform_name = "davinci-mcbsp.1",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
	.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM |
		   SND_SOC_DAIFMT_IB_NF,
};

static struct snd_soc_dai_link dm365_evm_dai = {
#ifdef CONFIG_SND_DM365_AIC3X_CODEC
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcbsp",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-0018",
	.platform_name = "davinci-mcbsp",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
	.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM |
		   SND_SOC_DAIFMT_IB_NF,
#elif defined(CONFIG_SND_DM365_VOICE_CODEC)
	.name = "Voice Codec - CQ93VC",
	.stream_name = "CQ93",
	.cpu_dai_name = "davinci-vcif",
	.codec_dai_name = "cq93vc-hifi",
	.codec_name = "cq93vc-codec",
	.platform_name = "davinci-vcif",
#endif
};

static struct snd_soc_dai_link dm6467_evm_dai[] = {
	{
		.name = "TLV320AIC3X",
		.stream_name = "AIC3X",
		.cpu_dai_name= "davinci-mcasp.0",
		.codec_dai_name = "tlv320aic3x-hifi",
		.platform_name = "davinci-mcasp.0",
		.codec_name = "tlv320aic3x-codec.0-001a",
		.init = evm_aic3x_init,
		.ops = &evm_ops,
		.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM |
			   SND_SOC_DAIFMT_IB_NF,
	},
	{
		.name = "McASP",
		.stream_name = "spdif",
		.cpu_dai_name= "davinci-mcasp.1",
		.codec_dai_name = "dit-hifi",
		.codec_name = "spdif_dit",
		.platform_name = "davinci-mcasp.1",
		.ops = &evm_spdif_ops,
		.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM |
			   SND_SOC_DAIFMT_IB_NF,
	},
};

static struct snd_soc_dai_link da830_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcasp.1",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-0018",
	.platform_name = "davinci-mcasp.1",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
	.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM |
		   SND_SOC_DAIFMT_IB_NF,
};

static struct snd_soc_dai_link da850_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name= "davinci-mcasp.0",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-0018",
	.platform_name = "davinci-mcasp.0",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
	.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM |
		   SND_SOC_DAIFMT_IB_NF,
};

/* davinci dm6446 evm audio machine driver */
/*
 * ASP0 in DM6446 EVM is clocked by U55, as configured by
 * board-dm644x-evm.c using GPIOs from U18.  There are six
 * options; here we "know" we use a 48 KHz sample rate.
 */
static struct snd_soc_card_drvdata_davinci dm6446_snd_soc_card_drvdata = {
	.sysclk = 12288000,
};

static struct snd_soc_card dm6446_snd_soc_card_evm = {
	.name = "DaVinci DM6446 EVM",
	.owner = THIS_MODULE,
	.dai_link = &dm6446_evm_dai,
	.num_links = 1,
	.drvdata = &dm6446_snd_soc_card_drvdata,
};

/* davinci dm355 evm audio machine driver */
/* ASP1 on DM355 EVM is clocked by an external oscillator */
static struct snd_soc_card_drvdata_davinci dm355_snd_soc_card_drvdata = {
	.sysclk = 27000000,
};

static struct snd_soc_card dm355_snd_soc_card_evm = {
	.name = "DaVinci DM355 EVM",
	.owner = THIS_MODULE,
	.dai_link = &dm355_evm_dai,
	.num_links = 1,
	.drvdata = &dm355_snd_soc_card_drvdata,
};

/* davinci dm365 evm audio machine driver */
static struct snd_soc_card_drvdata_davinci dm365_snd_soc_card_drvdata = {
	.sysclk = 27000000,
};

static struct snd_soc_card dm365_snd_soc_card_evm = {
	.name = "DaVinci DM365 EVM",
	.owner = THIS_MODULE,
	.dai_link = &dm365_evm_dai,
	.num_links = 1,
	.drvdata = &dm365_snd_soc_card_drvdata,
};

/* davinci dm6467 evm audio machine driver */
static struct snd_soc_card_drvdata_davinci dm6467_snd_soc_card_drvdata = {
	.sysclk = 27000000,
};

static struct snd_soc_card dm6467_snd_soc_card_evm = {
	.name = "DaVinci DM6467 EVM",
	.owner = THIS_MODULE,
	.dai_link = dm6467_evm_dai,
	.num_links = ARRAY_SIZE(dm6467_evm_dai),
	.drvdata = &dm6467_snd_soc_card_drvdata,
};

static struct snd_soc_card_drvdata_davinci da830_snd_soc_card_drvdata = {
	.sysclk = 24576000,
};

static struct snd_soc_card da830_snd_soc_card = {
	.name = "DA830/OMAP-L137 EVM",
	.owner = THIS_MODULE,
	.dai_link = &da830_evm_dai,
	.num_links = 1,
	.drvdata = &da830_snd_soc_card_drvdata,
};

static struct snd_soc_card_drvdata_davinci da850_snd_soc_card_drvdata = {
	.sysclk = 24576000,
};

static struct snd_soc_card da850_snd_soc_card = {
	.name = "DA850/OMAP-L138 EVM",
	.owner = THIS_MODULE,
	.dai_link = &da850_evm_dai,
	.num_links = 1,
	.drvdata = &da850_snd_soc_card_drvdata,
};

#if defined(CONFIG_OF)

/*
 * The structs are used as place holders. They will be completely
 * filled with data from dt node.
 */

static struct snd_soc_dai_link evm_dai_tlv320aic3x = {
	.name		= "TLV320AIC3X",	
	.stream_name	= "AIC3X",
	.codec_dai_name	= "tlv320aic3x-hifi",
	
	.ops            = &evm_ops,
	.init           = evm_aic3x_init,
	.dai_fmt 	= (SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_IB_NF),
	/*.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM |
		   SND_SOC_DAIFMT_IB_NF,*/
};

static struct snd_soc_dai_link evm_dai_tlv320aic32xx = {
	.name		= "TLV320AIC32XX",
	.stream_name	= "AIC3X",	
	.codec_dai_name	= "tlv320aic32x4-hifi",
	.ops            = &evm_ops,
	.init           = evm_aic3x_init,
	.dai_fmt 	= (SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_IB_NF),
	/*.dai_fmt 	= SND_SOC_DAIFMT_CBS_CFS, (SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_IB_NF),*/
};

static struct snd_soc_dai_link evm_dai_tda998x_hdmi = {
	.name		= "NXP TDA998x HDMI Chip",
	.stream_name	= "HDMI",
	.codec_dai_name	= "hdmi-hifi",
	.ops		= &evm_tda998x_ops,
	.init           = evm_tda998x_init,
	.dai_fmt	= (SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_IB_NF),
};

static struct snd_soc_dai_link dra7xx_evm_link = {
	.name		= "TLV320AIC3X",
	.stream_name	= "AIC3X",
	.codec_dai_name	= "tlv320aic3x-hifi",
	.platform_name = "omap-pcm-audio",
	.ops            = &dra7xx_ops,
	.init           = evm_aic3x_init,
	.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBS_CFS |
		   SND_SOC_DAIFMT_IB_NF,
};

static struct snd_soc_dai_link evm_dai_tlv320aic3111 = {
	.name		= "TLV320AIC3111",
	.stream_name	= "AIC3111",
	.codec_dai_name	= "tlv320aic31xx-hifi",
	.ops		= &evm_ops,
	.init		= evm_aic31xx_init,
	.dai_fmt	= (SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_DSP_B |
			   SND_SOC_DAIFMT_IB_NF),
};

static const struct of_device_id davinci_evm_dt_ids[] = {
	{
		.compatible = "ti,da830-evm-audio",
		.data = &evm_dai_tlv320aic3x,
	},
	{
		.compatible = "ti,tps2521-evm-audio",
		.data = &evm_dai_tlv320aic32xx,
	},
	{
		.compatible = "ti,am33xx-beaglebone-black",
		.data = &evm_dai_tda998x_hdmi,
	},
	{
		.compatible = "ti,dra7xx-evm-audio",
		.data = (void *) &dra7xx_evm_link,
	},
	{
		.compatible = "ti,am43xx-epos-evm-audio",
		.data = &evm_dai_tlv320aic3111,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, davinci_evm_dt_ids);

/* davinci evm audio machine driver */
static struct snd_soc_card evm_soc_card = {
	.owner = THIS_MODULE,
	.num_links = 1,
};

static int davinci_evm_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match =
		of_match_device(of_match_ptr(davinci_evm_dt_ids), &pdev->dev);
	struct snd_soc_dai_link *dai = (struct snd_soc_dai_link *) match->data;
	struct snd_soc_card_drvdata_davinci *drvdata = NULL;
	struct clk *mclk;
	int ret = 0;

	evm_soc_card.dai_link = dai;

	dai->codec_of_node = of_parse_phandle(np, "ti,audio-codec", 0);
	if (!dai->codec_of_node)
		return -EINVAL;

	dai->cpu_of_node = of_parse_phandle(np, "ti,mcasp-controller", 0);
	if (!dai->cpu_of_node)
		return -EINVAL;

	/* Only set the platform_of_node if the platform_name is not set */
	if (!dai->platform_name)
		dai->platform_of_node = dai->cpu_of_node;

	evm_soc_card.dev = &pdev->dev;
	
	ret = snd_soc_of_parse_card_name(&evm_soc_card, "ti,model");
	if (ret)
		return ret;
	
	mclk = of_clk_get_by_name(np, "ti,codec-clock");
	if (PTR_ERR(mclk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	else if (IS_ERR(mclk)) {
		dev_dbg(&pdev->dev, "Codec clock not found.\n");
		mclk = NULL;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->mclk = mclk;
	

	ret = of_property_read_u32(np, "ti,codec-clock-rate", &drvdata->sysclk);

	if (ret < 0) {
		if (!drvdata->mclk) {
			dev_err(&pdev->dev,
				"No clock or clock rate defined.\n");
			return -EINVAL;
		}
		drvdata->sysclk = clk_get_rate(drvdata->mclk);
	} else if (drvdata->mclk) {
		unsigned int requestd_rate = drvdata->sysclk;
		clk_set_rate(drvdata->mclk, drvdata->sysclk);
		drvdata->sysclk = clk_get_rate(drvdata->mclk);
		if (drvdata->sysclk != requestd_rate)
			dev_warn(&pdev->dev,
				 "Could not get requested rate %u using %u.\n",
				 requestd_rate, drvdata->sysclk);
	}
	
	snd_soc_card_set_drvdata(&evm_soc_card, drvdata);
	
	ret = snd_soc_register_card(&evm_soc_card);

	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);

	return ret;
}

static int davinci_evm_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct snd_soc_card_drvdata_davinci *drvdata =
		(struct snd_soc_card_drvdata_davinci *)
		snd_soc_card_get_drvdata(card);

	if (drvdata->mclk)
		clk_put(drvdata->mclk);

	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver davinci_evm_driver = {
	.probe		= davinci_evm_probe,
	.remove		= davinci_evm_remove,
	.driver		= {
		.name	= "davinci_evm",
		.owner	= THIS_MODULE,
		.pm	= &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(davinci_evm_dt_ids),
	},
};
#endif

static struct platform_device *evm_snd_device;

static int __init evm_init(void)
{
	struct snd_soc_card *evm_snd_dev_data;
	int index;
	int ret;

	/*
	 * If dtb is there, the devices will be created dynamically.
	 * Only register platfrom driver structure.
	 */
	if (of_have_populated_dt())
		return platform_driver_register(&davinci_evm_driver);

	if (machine_is_davinci_evm()) {
		evm_snd_dev_data = &dm6446_snd_soc_card_evm;
		index = 0;
	} else if (machine_is_davinci_dm355_evm()) {
		evm_snd_dev_data = &dm355_snd_soc_card_evm;
		index = 1;
	} else if (machine_is_davinci_dm365_evm()) {
		evm_snd_dev_data = &dm365_snd_soc_card_evm;
		index = 0;
	} else if (machine_is_davinci_dm6467_evm()) {
		evm_snd_dev_data = &dm6467_snd_soc_card_evm;
		index = 0;
	} else if (machine_is_davinci_da830_evm()) {
		evm_snd_dev_data = &da830_snd_soc_card;
		index = 1;
	} else if (machine_is_davinci_da850_evm()) {
		evm_snd_dev_data = &da850_snd_soc_card;
		index = 0;
	} else
		return -EINVAL;

	evm_snd_device = platform_device_alloc("soc-audio", index);
	if (!evm_snd_device)
		return -ENOMEM;

	platform_set_drvdata(evm_snd_device, evm_snd_dev_data);
	ret = platform_device_add(evm_snd_device);
	if (ret)
		platform_device_put(evm_snd_device);

	return ret;
}

static void __exit evm_exit(void)
{
	if (of_have_populated_dt()) {
		platform_driver_unregister(&davinci_evm_driver);
		return;
	}

	platform_device_unregister(evm_snd_device);
}

module_init(evm_init);
module_exit(evm_exit);

MODULE_AUTHOR("Vladimir Barinov");
MODULE_DESCRIPTION("TI DAVINCI EVM ASoC driver");
MODULE_LICENSE("GPL");
