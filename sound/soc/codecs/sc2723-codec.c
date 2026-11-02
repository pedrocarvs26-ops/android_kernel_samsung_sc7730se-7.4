// SPDX-License-Identifier: GPL-2.0
/*
 * Analog audio codec of the Spreadtrum SC2723 PMIC (companion of the
 * SC7730/SC8830 "shark" SoCs): DACs, the speaker mixer and the internal
 * speaker power amplifier. The digital side (VBC FIFO, DAC filters and
 * rate) lives in the SoC and is handled by the CPU DAI driver.
 *
 * Registers sit behind the PMIC's ADI bus: the analog block at 0x600, the
 * global audio enables at 0x800+. This driver is a child of the PMIC node
 * and uses its regmap. Sequences and bits from the vendor 3.10 driver
 * (sound/soc/sprd/codec/sprd/v4/sprd-codec.c), as summarised in
 * vendor-ref/research/audio-report.md of the bring-up repository.
 *
 * Outputs: the speaker (mono AOL output, class-AB PA fed from VBAT, DEMI
 * mode, as the vendor HAL configures it) and the headphones (HPL/HPR
 * through the class-G charge-pump amplifier, with the vendor's sequence and
 * HAL gains). Input: the main microphone into both ADCs. The headset
 * microphone, the auxiliary microphone and the analog loops are not wired
 * up; jack detection is a GPIO of the SoC (machine driver).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

/* SC2723 global registers */
#define GLB_ARM_MODULE_EN	0x800
#define  GLB_ANA_AUD_EN		BIT(4)
#define GLB_ARM_CLK_EN		0x804
#define  GLB_CLK_AUDIF_EN	BIT(0)
#define  GLB_CLK_AUD_6P5M_EN	BIT(1)
#define GLB_RTC_CLK_EN		0x808
#define  GLB_RTC_AUD_EN		BIT(4)
#define GLB_ARM_RST		0x80c
#define  GLB_AUD_RSTS		(BIT(9) | BIT(10) | BIT(11))
#define GLB_AUD_SLP_CTRL	0x8a4
#define GLB_XTL_WAIT_CTRL	0x8e8
#define  GLB_XTL_EN		BIT(8)
#define GLB_AUDIO_CTRL0		0x8fc
#define  GLB_6P5M_TX_INV	BIT(1)
#define AUDIFA_CTRL		0x400
#define  AUDIFA_ADSYNC_REUSE	BIT(4)

/* analog block */
#define ANA_PMU0		0x600
#define  PMU0_PA_SW_EN		BIT(15)
#define  PMU0_PA_LDO_EN		BIT(14)
#define  PMU0_PA_EN		BIT(13)
#define  PMU0_OVP_LDO_EN	BIT(11)
#define  PMU0_LDOCG_EN		BIT(10)		/* class-G LDO */
#define  PMU0_VB_EN		BIT(9)
#define  PMU0_VBO_EN		BIT(8)
#define ANA_PMU1		0x604
#define  PMU1_BIAS		GENMASK(15, 10)	/* BG, BG_IBIAS, VCM, VCM_BUF, SEL_VCMI, VCMI_FAST */
#define  PMU1_MICBIAS_EN	9
#define  PMU1_LDOCG_CTL		GENMASK(3, 1)	/* fall ILD, fall ISET, ramp */
#define ANA_PMU3		0x60c
#define  PMU3_PA_AB_I		GENMASK(13, 11)
#define ANA_PMU5		0x614
#define  PMU5_PA_OVP_V		GENMASK(15, 13)
#define ANA_PMU6		0x618
#define ANA_CDC0		0x61c
#define  CDC0_ADC_CLK_EN	BIT(15)
#define  CDC0_ADC_CLK_RST	BIT(14)
#define  CDC0_DAC_CLK_EN	11
#define  CDC0_DRV_CLK_EN	8
#define ANA_CDC1		0x620
#define  CDC1_ADCPGAL_EN	GENMASK(13, 12)
#define  CDC1_ADCPGAR_EN	GENMASK(9, 8)
#define  CDC1_ADC_IBUF_PD	7
#define  CDC1_ADCL_PD		5
#define  CDC1_ADCR_PD		3
#define ANA_CDC2		0x624
#define  CDC2_DACL_EN		15
#define  CDC2_DACR_EN		14
#define  CDC2_DACBUF_I_S	BIT(13)
#define  CDC2_DACDC_RMV_S	GENMASK(11, 10)
#define  CDC2_HPL_EN		8
#define  CDC2_HPR_EN		7
#define  CDC2_DIFF_EN		BIT(6)
#define  CDC2_AOL_EN		4
#define ANA_CDC3		0x628		/* class-G headphone amplifier */
#define  CDC3_CG_REF_EN		BIT(15)
#define  CDC3_CG_CHP_EN		BIT(14)
#define  CDC3_CG_HPL_EN		BIT(13)
#define  CDC3_CG_HPR_EN		BIT(12)
#define  CDC3_CG_CHP_OSC	GENMASK(9, 7)
#define  CDC3_CG_LPW		GENMASK(6, 5)
#define  CDC3_CG_HP_MODE	BIT(3)
#define  CDC3_CG_HPCAL_EN	BIT(2)
#define ANA_CDC4		0x62c
#define  CDC4_PA_D_EN		BIT(15)
#define  CDC4_PA_DTRI_F		GENMASK(14, 13)
#define  CDC4_PA_DEMI_EN	BIT(12)
#define ANA_CDC5		0x630		/* ADC input mixers */
#define  CDC5_MIC_ADCL		10
#define  CDC5_MIC_ADCR		15
#define ANA_CDC6		0x634
#define  CDC6_DACL_AOL		13
#define  CDC6_DACR_AOL		12
#define ANA_CDC7		0x638
#define  CDC7_HPL_CGL		BIT(11)		/* HP stage into class G */
#define  CDC7_HPR_CGR		BIT(10)
#define  CDC7_DACL_HPL		7
#define  CDC7_DACR_HPR		2
#define ANA_CDC8		0x63c		/* MIC boost 15:14 */
#define ANA_CDC9		0x640		/* ADC PGA L 9:4, R 15:10 */
#define ANA_CDC10		0x644	/* DAC gain L 15:13, R 12:10 */
#define ANA_CDC11		0x648	/* AOL gain 15:12 */
#define ANA_CDC12		0x64c	/* HPL gain 11:8, HPR 7:4 */
#define ANA_CDC13		0x650	/* class-G gains, L */
#define ANA_CDC14		0x654	/* class-G gains, R */
#define  CDC13_CG_G1		GENMASK(15, 10)
#define  CDC13_CG_G2		GENMASK(9, 7)
#define ANA_HDT1		0x66c
#define  HDT1_HEAD_L_INT_PU_PD	BIT(14)
#define ANA_DIG_CFG0		0x68c
#define  DIG_DACL_EN		BIT(9)
#define  DIG_ADCL_EN		BIT(10)
#define  DIG_DACR_EN		BIT(11)
#define  DIG_ADCR_EN		BIT(12)

/* vendor HAL "Inter HP PA Config" 0x1d41: oscillator 1, calibration on */
#define CG_OSC			1
#define CG_SWITCH_DELAY_MS	10
#define CG_PA_EN_DELAY_MS	130

struct sc2723_codec {
	struct regmap *regmap;
};

static int sc2723_analog_power_event(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(c, ANA_PMU0,
					      PMU0_VB_EN | PMU0_VBO_EN,
					      PMU0_VB_EN | PMU0_VBO_EN);
		snd_soc_component_update_bits(c, ANA_PMU1, PMU1_BIAS, PMU1_BIAS);
		snd_soc_component_update_bits(c, ANA_PMU0, PMU0_OVP_LDO_EN,
					      PMU0_OVP_LDO_EN);
		/* the vendor's SPRD_CODEC_LDO_WAIT_TIME */
		usleep_range(5000, 6000);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(c, ANA_PMU0, PMU0_OVP_LDO_EN, 0);
		snd_soc_component_update_bits(c, ANA_PMU1, PMU1_BIAS, 0);
		snd_soc_component_update_bits(c, ANA_PMU0,
					      PMU0_VB_EN | PMU0_VBO_EN, 0);
		break;
	}
	return 0;
}

/* the A-die digital DAC enables follow the analog DACs */
static int sc2723_dac_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	u32 bit = w->shift == CDC2_DACL_EN ? DIG_DACL_EN : DIG_DACR_EN;

	snd_soc_component_update_bits(c, ANA_DIG_CFG0, bit,
				      SND_SOC_DAPM_EVENT_ON(event) ? bit : 0);
	return 0;
}

static int sc2723_aol_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);

	/* the PA takes VBAT through PA_SW_EN, which follows the AOL output */
	snd_soc_component_update_bits(c, ANA_PMU0, PMU0_PA_SW_EN,
				      SND_SOC_DAPM_EVENT_ON(event) ?
				      PMU0_PA_SW_EN : 0);
	return 0;
}

/*
 * Internal speaker PA as the vendor HAL configures it ("Inter PA Config"
 * 0x130): class AB fed from VBAT through PA_SW_EN, DEMI mode, DTRI_F 3,
 * OVP at 4.96 V. Switched on last and off first.
 */
static int sc2723_spk_pa_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(c, ANA_PMU3, PMU3_PA_AB_I, 0);
		snd_soc_component_update_bits(c, ANA_PMU5, PMU5_PA_OVP_V,
					      FIELD_PREP(PMU5_PA_OVP_V, 3));
		snd_soc_component_update_bits(c, ANA_CDC4, CDC4_PA_D_EN, 0);
		snd_soc_component_write(c, ANA_PMU6, 0x0400);
		snd_soc_component_update_bits(c, ANA_CDC2,
					      CDC2_DACBUF_I_S | CDC2_DACDC_RMV_S,
					      CDC2_DACBUF_I_S |
					      FIELD_PREP(CDC2_DACDC_RMV_S, 3));
		snd_soc_component_update_bits(c, ANA_CDC4, CDC4_PA_DEMI_EN,
					      CDC4_PA_DEMI_EN);
		snd_soc_component_update_bits(c, ANA_PMU6,
					      BIT(15) | BIT(14), 0);
		snd_soc_component_update_bits(c, ANA_PMU0, PMU0_PA_LDO_EN, 0);
		snd_soc_component_update_bits(c, ANA_CDC4, CDC4_PA_DTRI_F,
					      FIELD_PREP(CDC4_PA_DTRI_F, 3));
		msleep(20);
		snd_soc_component_update_bits(c, ANA_PMU0, PMU0_PA_EN,
					      PMU0_PA_EN);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(c, ANA_PMU0, PMU0_PA_EN, 0);
		usleep_range(2000, 3000);
		break;
	}
	return 0;
}

/*
 * Class-G charge pump, LDO and reference, before the HP stages: vendor
 * sprd_inter_headphone_pa(), with the HAL's class-G gains
 */
static int sc2723_hp_cg_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	u32 gains = FIELD_PREP(CDC13_CG_G1, 63) | FIELD_PREP(CDC13_CG_G2, 7);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(c, ANA_CDC3,
					      CDC3_CG_LPW | CDC3_CG_CHP_OSC,
					      FIELD_PREP(CDC3_CG_CHP_OSC, CG_OSC));
		snd_soc_component_update_bits(c, ANA_CDC3, CDC3_CG_REF_EN,
					      CDC3_CG_REF_EN);
		snd_soc_component_update_bits(c, ANA_PMU0, PMU0_LDOCG_EN,
					      PMU0_LDOCG_EN);
		snd_soc_component_update_bits(c, ANA_CDC3, CDC3_CG_CHP_EN,
					      CDC3_CG_CHP_EN);
		snd_soc_component_update_bits(c, ANA_PMU1, PMU1_LDOCG_CTL,
					      PMU1_LDOCG_CTL);
		snd_soc_component_update_bits(c, ANA_HDT1, HDT1_HEAD_L_INT_PU_PD,
					      HDT1_HEAD_L_INT_PU_PD);
		snd_soc_component_update_bits(c, ANA_CDC13,
					      CDC13_CG_G1 | CDC13_CG_G2, gains);
		snd_soc_component_update_bits(c, ANA_CDC14,
					      CDC13_CG_G1 | CDC13_CG_G2, gains);
		msleep(CG_SWITCH_DELAY_MS);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(c, ANA_CDC3, CDC3_CG_REF_EN, 0);
		snd_soc_component_update_bits(c, ANA_PMU0, PMU0_LDOCG_EN, 0);
		snd_soc_component_update_bits(c, ANA_CDC3, CDC3_CG_CHP_EN, 0);
		snd_soc_component_update_bits(c, ANA_PMU1, PMU1_LDOCG_CTL, 0);
		break;
	}
	return 0;
}

/* differential output stage while a HP channel is on (vendor hp_switch_event) */
static int sc2723_hp_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);

	snd_soc_component_update_bits(c, ANA_CDC2, CDC2_DIFF_EN,
				      SND_SOC_DAPM_EVENT_ON(event) ?
				      CDC2_DIFF_EN : 0);
	return 0;
}

/* the class-G output stages, last on and first off (vendor ..._pa_post) */
static int sc2723_hp_pa_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(c, ANA_CDC3,
					      CDC3_CG_HP_MODE | CDC3_CG_HPCAL_EN,
					      CDC3_CG_HPCAL_EN);
		msleep(CG_PA_EN_DELAY_MS);
		snd_soc_component_update_bits(c, ANA_CDC3,
					      CDC3_CG_HPL_EN | CDC3_CG_HPR_EN,
					      CDC3_CG_HPL_EN | CDC3_CG_HPR_EN);
		snd_soc_component_update_bits(c, ANA_CDC7,
					      CDC7_HPL_CGL | CDC7_HPR_CGR,
					      CDC7_HPL_CGL | CDC7_HPR_CGR);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(c, ANA_CDC7,
					      CDC7_HPL_CGL | CDC7_HPR_CGR, 0);
		snd_soc_component_update_bits(c, ANA_CDC3,
					      CDC3_CG_HPL_EN | CDC3_CG_HPR_EN, 0);
		msleep(10);
		snd_soc_component_update_bits(c, ANA_CDC3, CDC3_CG_HPCAL_EN, 0);
		break;
	}
	return 0;
}

/* ADC clock with a reset pulse (vendor sprd_codec_adc_clock_input_en) */
static int sc2723_adc_clk_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		snd_soc_component_update_bits(c, ANA_CDC0, CDC0_ADC_CLK_EN,
					      CDC0_ADC_CLK_EN);
		snd_soc_component_update_bits(c, ANA_CDC0, CDC0_ADC_CLK_RST,
					      CDC0_ADC_CLK_RST);
		snd_soc_component_update_bits(c, ANA_CDC0, CDC0_ADC_CLK_RST, 0);
	} else {
		snd_soc_component_update_bits(c, ANA_CDC0, CDC0_ADC_CLK_EN, 0);
	}
	return 0;
}

static int sc2723_adc_pga_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	u32 mask = w->shift ? CDC1_ADCPGAR_EN : CDC1_ADCPGAL_EN;

	snd_soc_component_update_bits(c, ANA_CDC1, mask,
				      SND_SOC_DAPM_EVENT_ON(event) ? mask : 0);
	return 0;
}

/* the A-die digital ADC enables follow the analog ADCs */
static int sc2723_adc_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	u32 bit = w->shift == CDC1_ADCL_PD ? DIG_ADCL_EN : DIG_ADCR_EN;

	snd_soc_component_update_bits(c, ANA_DIG_CFG0, bit,
				      SND_SOC_DAPM_EVENT_ON(event) ? bit : 0);
	return 0;
}

static const DECLARE_TLV_DB_SCALE(sc2723_dac_tlv, -1050, 150, 0);
static const DECLARE_TLV_DB_SCALE(sc2723_aol_tlv, -2400, 300, 0);
/*
 * Headphone, mic boost and ADC scales as in the vendor driver; the ADC step
 * (0.75 dB) is confirmed by measurement, the mic boost steps measured
 * smaller than the nominal 6 dB (about 4 dB each).
 */
static const DECLARE_TLV_DB_SCALE(sc2723_hp_tlv, -3600, 300, 1);
static const DECLARE_TLV_DB_SCALE(sc2723_mic_tlv, 0, 600, 0);
static const DECLARE_TLV_DB_SCALE(sc2723_adc_tlv, -600, 75, 0);

static const struct snd_kcontrol_new sc2723_controls[] = {
	SOC_DOUBLE_TLV("DAC Playback Volume", ANA_CDC10, 13, 10, 7, 0,
		       sc2723_dac_tlv),
	SOC_SINGLE_TLV("Speaker Playback Volume", ANA_CDC11, 12, 15, 0,
		       sc2723_aol_tlv),
	SOC_DOUBLE_TLV("Headphone Playback Volume", ANA_CDC12, 8, 4, 15, 0,
		       sc2723_hp_tlv),
	SOC_SINGLE_TLV("Mic Boost Volume", ANA_CDC8, 14, 3, 0, sc2723_mic_tlv),
	SOC_DOUBLE_TLV("ADC Capture Volume", ANA_CDC9, 4, 10, 63, 0,
		       sc2723_adc_tlv),
};

static const struct snd_kcontrol_new sc2723_hpl_mixer[] = {
	SOC_DAPM_SINGLE("DACL Switch", ANA_CDC7, CDC7_DACL_HPL, 1, 0),
};

static const struct snd_kcontrol_new sc2723_hpr_mixer[] = {
	SOC_DAPM_SINGLE("DACR Switch", ANA_CDC7, CDC7_DACR_HPR, 1, 0),
};

static const struct snd_kcontrol_new sc2723_adcl_mixer[] = {
	SOC_DAPM_SINGLE("MIC Switch", ANA_CDC5, CDC5_MIC_ADCL, 1, 0),
};

static const struct snd_kcontrol_new sc2723_adcr_mixer[] = {
	SOC_DAPM_SINGLE("MIC Switch", ANA_CDC5, CDC5_MIC_ADCR, 1, 0),
};

static const struct snd_kcontrol_new sc2723_spk_mixer[] = {
	SOC_DAPM_SINGLE("DACL Switch", ANA_CDC6, CDC6_DACL_AOL, 1, 0),
	SOC_DAPM_SINGLE("DACR Switch", ANA_CDC6, CDC6_DACR_AOL, 1, 0),
};

static const struct snd_soc_dapm_widget sc2723_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY_S("Analog Power", 1, SND_SOC_NOPM, 0, 0,
			      sc2723_analog_power_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("DA Clk", 2, ANA_CDC0, CDC0_DAC_CLK_EN, 0,
			      NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("DRV Clk", 2, ANA_CDC0, CDC0_DRV_CLK_EN, 0,
			      NULL, 0),
	SND_SOC_DAPM_DAC_E("DACL", "Playback", ANA_CDC2, CDC2_DACL_EN, 0,
			   sc2723_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("DACR", "Playback", ANA_CDC2, CDC2_DACR_EN, 0,
			   sc2723_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("SPKL Mixer", SND_SOC_NOPM, 0, 0,
			   sc2723_spk_mixer, ARRAY_SIZE(sc2723_spk_mixer)),
	SND_SOC_DAPM_PGA_E("AOL", ANA_CDC2, CDC2_AOL_EN, 0, NULL, 0,
			   sc2723_aol_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("Spk PA", SND_SOC_NOPM, 0, 0, NULL, 0,
			       sc2723_spk_pa_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("SPK"),

	SND_SOC_DAPM_SUPPLY_S("HP Class-G", 3, SND_SOC_NOPM, 0, 0,
			      sc2723_hp_cg_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("HPL Mixer", SND_SOC_NOPM, 0, 0,
			   sc2723_hpl_mixer, ARRAY_SIZE(sc2723_hpl_mixer)),
	SND_SOC_DAPM_MIXER("HPR Mixer", SND_SOC_NOPM, 0, 0,
			   sc2723_hpr_mixer, ARRAY_SIZE(sc2723_hpr_mixer)),
	SND_SOC_DAPM_PGA_E("HPL", ANA_CDC2, CDC2_HPL_EN, 0, NULL, 0,
			   sc2723_hp_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("HPR", ANA_CDC2, CDC2_HPR_EN, 0, NULL, 0,
			   sc2723_hp_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("HP PA", SND_SOC_NOPM, 0, 0, NULL, 0,
			       sc2723_hp_pa_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("HPOL"),
	SND_SOC_DAPM_OUTPUT("HPOR"),

	SND_SOC_DAPM_INPUT("MIC"),
	SND_SOC_DAPM_SUPPLY("Mic Bias", ANA_PMU1, PMU1_MICBIAS_EN, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("AD Clk", 2, SND_SOC_NOPM, 0, 0,
			      sc2723_adc_clk_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("AD IBUF", 2, ANA_CDC1, CDC1_ADC_IBUF_PD, 1,
			      NULL, 0),
	SND_SOC_DAPM_MIXER("ADCL Mixer", SND_SOC_NOPM, 0, 0,
			   sc2723_adcl_mixer, ARRAY_SIZE(sc2723_adcl_mixer)),
	SND_SOC_DAPM_MIXER("ADCR Mixer", SND_SOC_NOPM, 0, 0,
			   sc2723_adcr_mixer, ARRAY_SIZE(sc2723_adcr_mixer)),
	SND_SOC_DAPM_PGA_E("ADCL PGA", SND_SOC_NOPM, 0, 0, NULL, 0,
			   sc2723_adc_pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("ADCR PGA", SND_SOC_NOPM, 1, 0, NULL, 0,
			   sc2723_adc_pga_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADCL", "Capture", ANA_CDC1, CDC1_ADCL_PD, 1,
			   sc2723_adc_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_ADC_E("ADCR", "Capture", ANA_CDC1, CDC1_ADCR_PD, 1,
			   sc2723_adc_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
};

static const struct snd_soc_dapm_route sc2723_dapm_routes[] = {
	{ "DACL", NULL, "Analog Power" },
	{ "DACR", NULL, "Analog Power" },
	{ "DACL", NULL, "DA Clk" },
	{ "DACR", NULL, "DA Clk" },
	{ "DACL", NULL, "DRV Clk" },
	{ "DACR", NULL, "DRV Clk" },
	{ "SPKL Mixer", "DACL Switch", "DACL" },
	{ "SPKL Mixer", "DACR Switch", "DACR" },
	{ "AOL", NULL, "SPKL Mixer" },
	{ "Spk PA", NULL, "AOL" },
	{ "SPK", NULL, "Spk PA" },

	{ "HPL Mixer", "DACL Switch", "DACL" },
	{ "HPR Mixer", "DACR Switch", "DACR" },
	{ "HPL", NULL, "HPL Mixer" },
	{ "HPR", NULL, "HPR Mixer" },
	{ "HPL", NULL, "HP Class-G" },
	{ "HPR", NULL, "HP Class-G" },
	{ "HP PA", NULL, "HPL" },
	{ "HP PA", NULL, "HPR" },
	{ "HPOL", NULL, "HP PA" },
	{ "HPOR", NULL, "HP PA" },

	{ "MIC", NULL, "Mic Bias" },
	{ "ADCL Mixer", "MIC Switch", "MIC" },
	{ "ADCR Mixer", "MIC Switch", "MIC" },
	{ "ADCL PGA", NULL, "ADCL Mixer" },
	{ "ADCR PGA", NULL, "ADCR Mixer" },
	{ "ADCL", NULL, "ADCL PGA" },
	{ "ADCR", NULL, "ADCR PGA" },
	{ "ADCL", NULL, "Analog Power" },
	{ "ADCR", NULL, "Analog Power" },
	{ "ADCL", NULL, "AD Clk" },
	{ "ADCR", NULL, "AD Clk" },
	{ "ADCL", NULL, "AD IBUF" },
	{ "ADCR", NULL, "AD IBUF" },
};

/*
 * Global audio enables of the PMIC, kept on while the codec is bound:
 * module enable, no sleep power-down, one analog reset, AUDIF and 6.5 MHz
 * clocks with TX inversion, no ADSYNC reuse, 32 kHz and crystal.
 */
static int sc2723_codec_component_probe(struct snd_soc_component *c)
{
	struct sc2723_codec *codec = snd_soc_component_get_drvdata(c);

	/* the registers belong to the PMIC's regmap */
	snd_soc_component_init_regmap(c, codec->regmap);
	snd_soc_component_update_bits(c, GLB_ARM_MODULE_EN, GLB_ANA_AUD_EN,
				      GLB_ANA_AUD_EN);
	snd_soc_component_write(c, GLB_AUD_SLP_CTRL, 0);
	snd_soc_component_update_bits(c, GLB_ARM_RST, GLB_AUD_RSTS,
				      GLB_AUD_RSTS);
	udelay(10);
	snd_soc_component_update_bits(c, GLB_ARM_RST, GLB_AUD_RSTS, 0);
	snd_soc_component_update_bits(c, GLB_ARM_CLK_EN,
				      GLB_CLK_AUDIF_EN | GLB_CLK_AUD_6P5M_EN,
				      GLB_CLK_AUDIF_EN | GLB_CLK_AUD_6P5M_EN);
	snd_soc_component_update_bits(c, GLB_AUDIO_CTRL0, GLB_6P5M_TX_INV,
				      GLB_6P5M_TX_INV);
	snd_soc_component_update_bits(c, AUDIFA_CTRL, AUDIFA_ADSYNC_REUSE, 0);
	snd_soc_component_update_bits(c, GLB_RTC_CLK_EN, GLB_RTC_AUD_EN,
				      GLB_RTC_AUD_EN);
	snd_soc_component_update_bits(c, GLB_XTL_WAIT_CTRL, GLB_XTL_EN,
				      GLB_XTL_EN);
	/* both DACs into the mono speaker, each into its HP side */
	snd_soc_component_update_bits(c, ANA_CDC6,
				      BIT(CDC6_DACL_AOL) | BIT(CDC6_DACR_AOL),
				      BIT(CDC6_DACL_AOL) | BIT(CDC6_DACR_AOL));
	snd_soc_component_update_bits(c, ANA_CDC7,
				      BIT(CDC7_DACL_HPL) | BIT(CDC7_DACR_HPR),
				      BIT(CDC7_DACL_HPL) | BIT(CDC7_DACR_HPR));
	/* main microphone into both ADCs; HAL gains: HP 8, ADC PGA 12 */
	snd_soc_component_update_bits(c, ANA_CDC5,
				      BIT(CDC5_MIC_ADCL) | BIT(CDC5_MIC_ADCR),
				      BIT(CDC5_MIC_ADCL) | BIT(CDC5_MIC_ADCR));
	snd_soc_component_update_bits(c, ANA_CDC12, GENMASK(11, 4),
				      8 << 8 | 8 << 4);
	snd_soc_component_update_bits(c, ANA_CDC9, GENMASK(15, 4),
				      12 << 10 | 12 << 4);
	return 0;
}

static void sc2723_codec_component_remove(struct snd_soc_component *c)
{
	snd_soc_component_update_bits(c, GLB_ARM_CLK_EN,
				      GLB_CLK_AUDIF_EN | GLB_CLK_AUD_6P5M_EN, 0);
	snd_soc_component_update_bits(c, GLB_ARM_MODULE_EN, GLB_ANA_AUD_EN, 0);
}

static const struct snd_soc_component_driver sc2723_codec_driver = {
	.probe = sc2723_codec_component_probe,
	.remove = sc2723_codec_component_remove,
	.controls = sc2723_controls,
	.num_controls = ARRAY_SIZE(sc2723_controls),
	.dapm_widgets = sc2723_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sc2723_dapm_widgets),
	.dapm_routes = sc2723_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(sc2723_dapm_routes),
	.idle_bias_on = false,
	.use_pmdown_time = true,
	.endianness = 1,
};

/* the link to the SoC's DAC filters is internal: no format to set */
static struct snd_soc_dai_driver sc2723_dai = {
	.name = "sc2723-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static int sc2723_codec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc2723_codec *codec;

	codec = devm_kzalloc(dev, sizeof(*codec), GFP_KERNEL);
	if (!codec)
		return -ENOMEM;
	codec->regmap = dev_get_regmap(dev->parent, NULL);
	if (!codec->regmap)
		return dev_err_probe(dev, -ENODEV, "no PMIC regmap\n");
	platform_set_drvdata(pdev, codec);

	return devm_snd_soc_register_component(dev, &sc2723_codec_driver,
					       &sc2723_dai, 1);
}

static const struct of_device_id sc2723_codec_of_match[] = {
	{ .compatible = "sprd,sc2723-codec" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc2723_codec_of_match);

static struct platform_driver sc2723_codec_platform_driver = {
	.probe = sc2723_codec_probe,
	.driver = {
		.name = "sc2723-codec",
		.of_match_table = sc2723_codec_of_match,
	},
};
module_platform_driver(sc2723_codec_platform_driver);

MODULE_DESCRIPTION("Spreadtrum SC2723 PMIC audio codec");
MODULE_LICENSE("GPL");
