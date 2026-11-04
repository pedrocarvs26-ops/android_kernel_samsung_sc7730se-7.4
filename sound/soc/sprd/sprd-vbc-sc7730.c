// SPDX-License-Identifier: GPL-2.0
/*
 * Voice band codec (VBC r2p0) of the Spreadtrum SC7730/SC8830 ("shark")
 * SoCs as a CPU DAI, together with the "DP", the SoC-side digital part of
 * the audio codec (DAC filters, sample rate, sigma-delta modulator) that
 * feeds the analog codec in the SC2723 PMIC over the internal AUDIF link.
 *
 * The VBC has two DA FIFOs (left/right, 16-bit samples) with ping-pong
 * halves of 160 frames; the AP DMA fills them on VBC requests (UIDs 29/30),
 * which sound/soc/sprd/sprd-pcm-dma.c drives with the parameters set here.
 * Capture reads the AD0/AD1 FIFOs the same way (UIDs 31/32).
 * Register sequences from the vendor 3.10 drivers (sound/soc/sprd/dai/vbc/
 * r2p0, codec/sprd/v4), summarised in vendor-ref/research/audio-report.md
 * of the bring-up repository.
 *
 * The bootloader leaves the VBC owned by the modem side; probe hands it and
 * the codec interrupts/DMA requests to the AP.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "sprd-pcm-dma.h"

/* VBC */
#define VBDA0			0x00
#define VBDA1			0x04
#define VBAD0			0x08
#define VBAD1			0x0c
#define VBBUFFSIZE		0x10
#define VBDABUFFDTA		0x18
#define  RAMSW_NUMB		BIT(9)
#define  RAMSW_EN		BIT(10)
#define  AD0_DMA_EN		BIT(11)
#define  AD1_DMA_EN		BIT(12)
#define  DA0_DMA_EN		BIT(13)
#define  DA1_DMA_EN		BIT(14)
#define  VBENABLE		BIT(15)
#define VBIISSEL		0x3c
#define  DA_LRCK_POL		BIT(14)
/*
 * AD0/AD1 digital gain: 7-bit attenuation in 0.75 dB steps with 0x18 =
 * 0 dB (measured on the gtelwifi: 0x00 +18 dB, 0x30 -18 dB) and an enable
 * bit per channel. Only 0x00-0x30 is exposed.
 */
#define ADDG01CTL		0x84
#define  AD0_DG_EN		BIT(7)
#define  AD1_DG_EN		BIT(15)
#define  AD_DG_0DB		0x18
#define  AD_DG_MAX		0x30
#define DAHPCTL			0x48
/*
 * Bits 8:0 of DAHPCTL as the vendor's neutral DA profile sets them; with 0
 * the DA path comes out quiet and grainy. Bits 10/11 (EQ6/ALC) stay off.
 */
#define DAHPCTL_DEFAULT		0x17f
#define VBCHNEN			0xc8
#define  DA0_CHN_EN		BIT(0)
#define  DA1_CHN_EN		BIT(1)
#define  AD0_CHN_EN		BIT(2)
#define  AD1_CHN_EN		BIT(3)
#define VBC_FIFO_FRAMES		160

/* DP */
#define AUD_TOP_CTL		0x00
#define  DAC_EN_L		BIT(0)
#define  ADC_EN_L		BIT(1)
#define  DAC_EN_R		BIT(2)
#define  ADC_EN_R		BIT(3)
#define AUD_DAC_CTL		0x0c
#define  DAC_RATE		GENMASK(3, 0)
#define  DAC_MUTE_EN		BIT(15)
#define AUD_SDM_CTL0		0x10
#define  SDM_DODVL		GENMASK(11, 8)
#define AUD_ADC_CTL		0x18
#define  ADC_RATE		GENMASK(3, 0)	/* rate / 4000 */

/* AON APB */
#define AON_EB0			0x00
#define  EB0_VBC		BIT(19)
#define  EB0_AUD		BIT(18)
#define AON_RST0		0x08
#define  RST0_AUD		(BIT(20) | BIT(19) | BIT(18))
#define AON_SINDRV_CTRL		0x18
#define  SINDRV_ENA		BIT(0)
#define AON_VBC_CTRL		0x20
#define  VBC_AUDIF_CKG_AUTO	BIT(20)
#define  VBC_SYS_SEL		GENMASK(19, 0)	/* 0 = AP for all */
#define AON_AP_WPROT_EN		0x3040
#define AON_CP_WPROT_EN(n)	(0x3044 + 4 * (n))
#define  WPROT_VBC		BIT(6)

static const struct {
	unsigned int rate;
	u32 code;
} sprd_vbc_rates[] = {
	{ 96000, 0 }, { 48000, 1 }, { 44100, 2 }, { 32000, 3 }, { 24000, 4 },
	{ 22050, 5 }, { 16000, 6 }, { 12000, 7 }, { 11025, 8 }, { 8000, 10 },
};

struct sprd_vbc {
	struct device *dev;
	void __iomem *vbc, *dp, *aon;
	struct sprd_pcm_dma_params dma[2];	/* playback, capture */
	unsigned int channels[2];
	spinlock_t lock;			/* VBC/DP registers in trigger */
	unsigned int running;			/* streams holding VBENABLE */
};

/* AON APB is read-modify-written by other bring-up glue too */
static DEFINE_SPINLOCK(sprd_vbc_aon_lock);

static void sprd_vbc_rmw(void __iomem *reg, u32 clr, u32 set)
{
	writel((readl(reg) & ~clr) | set, reg);
}

static void sprd_vbc_aon_rmw(struct sprd_vbc *v, u32 off, u32 clr, u32 set)
{
	unsigned long flags;

	spin_lock_irqsave(&sprd_vbc_aon_lock, flags);
	sprd_vbc_rmw(v->aon + off, clr, set);
	spin_unlock_irqrestore(&sprd_vbc_aon_lock, flags);
}

/*
 * The VBC replays its DA RAM when nothing refills it, so both ping-pong
 * halves are cleared before and after every stream ("vbc da close MUST
 * clear da buffer" in the vendor driver).
 */
static void sprd_vbc_clear_da(struct sprd_vbc *v)
{
	int half, i;

	for (half = 0; half < 2; half++) {
		sprd_vbc_rmw(v->vbc + VBDABUFFDTA, RAMSW_NUMB,
			     RAMSW_EN | (half ? RAMSW_NUMB : 0));
		for (i = 0; i < VBC_FIFO_FRAMES; i++) {
			writew(0, v->vbc + VBDA0);
			writew(0, v->vbc + VBDA1);
		}
	}
	sprd_vbc_rmw(v->vbc + VBDABUFFDTA, RAMSW_EN | RAMSW_NUMB, 0);
}

static int sprd_vbc_startup(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct sprd_vbc *v = snd_soc_dai_get_drvdata(dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		sprd_vbc_clear_da(v);
	writel((VBC_FIFO_FRAMES - 1) << 8 | (VBC_FIFO_FRAMES - 1),
	       v->vbc + VBBUFFSIZE);
	/* DA and AD01 port 0 = codec */
	writel(DA_LRCK_POL, v->vbc + VBIISSEL);
	writel(DAHPCTL_DEFAULT, v->vbc + DAHPCTL);	/* EQ and ALC off */
	snd_soc_dai_set_dma_data(dai, substream, &v->dma[substream->stream]);
	return 0;
}

static void sprd_vbc_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		sprd_vbc_clear_da(snd_soc_dai_get_drvdata(dai));
}

static int sprd_vbc_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	struct sprd_vbc *v = snd_soc_dai_get_drvdata(dai);
	unsigned int rate = params_rate(params);
	int i;

	v->channels[substream->stream] = params_channels(params);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (rate % 4000)
			return -EINVAL;
		sprd_vbc_rmw(v->dp + AUD_ADC_CTL, ADC_RATE,
			     FIELD_PREP(ADC_RATE, rate / 4000));
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(sprd_vbc_rates); i++)
		if (sprd_vbc_rates[i].rate == rate)
			break;
	if (i == ARRAY_SIZE(sprd_vbc_rates))
		return -EINVAL;

	sprd_vbc_rmw(v->dp + AUD_DAC_CTL, DAC_RATE,
		     FIELD_PREP(DAC_RATE, sprd_vbc_rates[i].code));
	sprd_vbc_rmw(v->dp + AUD_SDM_CTL0, SDM_DODVL,
		     FIELD_PREP(SDM_DODVL, 7));
	return 0;
}

static int sprd_vbc_trigger(struct snd_pcm_substream *substream, int cmd,
			    struct snd_soc_dai *dai)
{
	struct sprd_vbc *v = snd_soc_dai_get_drvdata(dai);
	bool capture = substream->stream == SNDRV_PCM_STREAM_CAPTURE;
	bool stereo = v->channels[substream->stream] > 1;
	u32 chn, dma, top;
	unsigned long flags;

	if (capture) {
		chn = AD0_CHN_EN | (stereo ? AD1_CHN_EN : 0);
		dma = AD0_DMA_EN | (stereo ? AD1_DMA_EN : 0);
		top = ADC_EN_L | ADC_EN_R;
	} else {
		chn = DA0_CHN_EN | (stereo ? DA1_CHN_EN : 0);
		dma = DA0_DMA_EN | (stereo ? DA1_DMA_EN : 0);
		top = DAC_EN_L | DAC_EN_R;
	}

	spin_lock_irqsave(&v->lock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (!capture)
			sprd_vbc_rmw(v->dp + AUD_DAC_CTL, DAC_MUTE_EN, 0);
		sprd_vbc_rmw(v->dp + AUD_TOP_CTL, 0, top);
		sprd_vbc_rmw(v->vbc + VBCHNEN, 0, chn);
		sprd_vbc_rmw(v->vbc + VBDABUFFDTA, 0, dma);
		if (!v->running++)
			sprd_vbc_rmw(v->vbc + VBDABUFFDTA, 0, VBENABLE);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/*
		 * The digital DACs stay on: the analog path behind them can
		 * stay powered between streams (pmdown_time), and an analog
		 * DAC without a running sigma-delta modulator picks up noise
		 * that the speaker amplifier makes audible. They go off with
		 * the system suspend, like the analog path. The ADCs go off
		 * with their stream.
		 */
		if (capture) {
			chn = AD0_CHN_EN | AD1_CHN_EN;
			dma = AD0_DMA_EN | AD1_DMA_EN;
			sprd_vbc_rmw(v->dp + AUD_TOP_CTL, top, 0);
		} else {
			chn = DA0_CHN_EN | DA1_CHN_EN;
			dma = DA0_DMA_EN | DA1_DMA_EN;
		}
		if (v->running && !--v->running)
			dma |= VBENABLE;
		sprd_vbc_rmw(v->vbc + VBDABUFFDTA, dma, 0);
		sprd_vbc_rmw(v->vbc + VBCHNEN, chn, 0);
		break;
	default:
		spin_unlock_irqrestore(&v->lock, flags);
		return -EINVAL;
	}
	spin_unlock_irqrestore(&v->lock, flags);
	return 0;
}

static const struct snd_soc_dai_ops sprd_vbc_dai_ops = {
	.startup = sprd_vbc_startup,
	.shutdown = sprd_vbc_shutdown,
	.hw_params = sprd_vbc_hw_params,
	.trigger = sprd_vbc_trigger,
};

/*
 * DA0/AD0 = left, DA1/AD1 = right, one DMA channel each. The name is kept
 * from the playback-only days: userspace sets pmdown_time through it.
 */
static struct snd_soc_dai_driver sprd_vbc_dai = {
	.name = "vbc-playback",
	.playback = {
		.stream_name = "VBC Playback",
		.channels_min = 1,
		.channels_max = 2,
		/* exactly the DAC_RATE codes in sprd_vbc_rates[] */
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
			 SNDRV_PCM_RATE_12000 | SNDRV_PCM_RATE_16000 |
			 SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_24000 |
			 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "VBC Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &sprd_vbc_dai_ops,
};

static int sprd_vbc_suspend(struct snd_soc_component *component)
{
	struct sprd_vbc *v = snd_soc_component_get_drvdata(component);

	sprd_vbc_rmw(v->dp + AUD_TOP_CTL,
		     DAC_EN_L | DAC_EN_R | ADC_EN_L | ADC_EN_R, 0);
	return 0;
}

/* the controls below only reach VBC registers, which stay clocked */
static unsigned int sprd_vbc_read(struct snd_soc_component *component,
				  unsigned int reg)
{
	struct sprd_vbc *v = snd_soc_component_get_drvdata(component);

	return readl(v->vbc + reg);
}

static int sprd_vbc_write(struct snd_soc_component *component,
			  unsigned int reg, unsigned int val)
{
	struct sprd_vbc *v = snd_soc_component_get_drvdata(component);

	writel(val, v->vbc + reg);
	return 0;
}

static const DECLARE_TLV_DB_SCALE(sprd_vbc_ad_dg_tlv, -1800, 75, 0);

static const struct snd_kcontrol_new sprd_vbc_controls[] = {
	SOC_DOUBLE_TLV("ADC Digital Capture Volume", ADDG01CTL, 0, 8,
		       AD_DG_MAX, 1, sprd_vbc_ad_dg_tlv),
};

static const struct snd_soc_component_driver sprd_vbc_component = {
	.name = "sprd-vbc-sc7730",
	.suspend = sprd_vbc_suspend,
	.read = sprd_vbc_read,
	.write = sprd_vbc_write,
	.controls = sprd_vbc_controls,
	.num_controls = ARRAY_SIZE(sprd_vbc_controls),
	.legacy_dai_naming = 1,
};

static void __iomem *sprd_vbc_map(struct platform_device *pdev,
				  const char *name)
{
	struct resource *res;

	/* AON APB is shared with other drivers: no request */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return NULL;
	return devm_ioremap(&pdev->dev, res->start, resource_size(res));
}

static int sprd_vbc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct sprd_vbc *v;
	int i;

	v = devm_kzalloc(dev, sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;
	v->dev = dev;
	spin_lock_init(&v->lock);
	v->aon = sprd_vbc_map(pdev, "aon-apb");
	v->vbc = sprd_vbc_map(pdev, "vbc");
	v->dp = sprd_vbc_map(pdev, "dp");
	if (!v->aon || !v->vbc || !v->dp)
		return -ENOMEM;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "vbc");

	/* clocks before any VBC or DP access: gated APB slaves hang the bus */
	sprd_vbc_aon_rmw(v, AON_EB0, 0, EB0_VBC | EB0_AUD);
	/* take VBC, codec interrupts and DMA requests from the modem side */
	sprd_vbc_aon_rmw(v, AON_VBC_CTRL, VBC_SYS_SEL, VBC_AUDIF_CKG_AUTO);
	sprd_vbc_aon_rmw(v, AON_AP_WPROT_EN, WPROT_VBC, 0);
	for (i = 0; i < 3; i++)
		sprd_vbc_aon_rmw(v, AON_CP_WPROT_EN(i), 0, WPROT_VBC);
	sprd_vbc_aon_rmw(v, AON_SINDRV_CTRL, 0, SINDRV_ENA);
	sprd_vbc_aon_rmw(v, AON_RST0, 0, RST0_AUD);
	udelay(10);
	sprd_vbc_aon_rmw(v, AON_RST0, RST0_AUD, 0);
	sprd_vbc_clear_da(v);
	/* AD digital gain on at 0 dB, so the control below sets it directly */
	writel(AD0_DG_EN | AD1_DG_EN | AD_DG_0DB << 8 | AD_DG_0DB,
	       v->vbc + ADDG01CTL);

	for (i = 0; i < 2; i++) {
		v->dma[SNDRV_PCM_STREAM_PLAYBACK].dev_phys[i] =
			res->start + (i ? VBDA1 : VBDA0);
		v->dma[SNDRV_PCM_STREAM_CAPTURE].dev_phys[i] =
			res->start + (i ? VBAD1 : VBAD0);
		v->dma[SNDRV_PCM_STREAM_PLAYBACK].datawidth[i] =
			DMA_SLAVE_BUSWIDTH_2_BYTES;
		v->dma[SNDRV_PCM_STREAM_CAPTURE].datawidth[i] =
			DMA_SLAVE_BUSWIDTH_2_BYTES;
		v->dma[SNDRV_PCM_STREAM_PLAYBACK].fragment_len[i] =
			VBC_FIFO_FRAMES * 2;
		v->dma[SNDRV_PCM_STREAM_CAPTURE].fragment_len[i] =
			VBC_FIFO_FRAMES * 2;
	}
	v->dma[SNDRV_PCM_STREAM_PLAYBACK].chan_name[0] = "normal_p_l";
	v->dma[SNDRV_PCM_STREAM_PLAYBACK].chan_name[1] = "normal_p_r";
	v->dma[SNDRV_PCM_STREAM_CAPTURE].chan_name[0] = "normal_c_l";
	v->dma[SNDRV_PCM_STREAM_CAPTURE].chan_name[1] = "normal_c_r";

	platform_set_drvdata(pdev, v);
	return devm_snd_soc_register_component(dev, &sprd_vbc_component,
					       &sprd_vbc_dai, 1);
}

static const struct of_device_id sprd_vbc_of_match[] = {
	{ .compatible = "sprd,sc7730-vbc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sprd_vbc_of_match);

static struct platform_driver sprd_vbc_driver = {
	.probe = sprd_vbc_probe,
	.driver = {
		.name = "sprd-vbc-sc7730",
		.of_match_table = sprd_vbc_of_match,
	},
};
module_platform_driver(sprd_vbc_driver);

MODULE_DESCRIPTION("Spreadtrum SC7730 VBC and audio DP CPU DAI");
MODULE_LICENSE("GPL");
