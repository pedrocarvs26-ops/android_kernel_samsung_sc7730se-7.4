// SPDX-License-Identifier: GPL-2.0
/*
 * PWM block of the Spreadtrum SC8830/SC7730 ("scx35") always-on domain.
 *
 * Four channels, 0x20 apart, each with an 8-bit prescaler and an 8-bit
 * counter: PRESCALE (bit 8 enable, bits 7:0 prescale), CNT (bits 15:8
 * duty, bits 7:0 modulus) and two pattern registers that have to read
 * all ones for a plain PWM. Period = (prescale + 1) * mod / rate and
 * duty = (prescale + 1) * duty / rate, the same counter as in mainline
 * pwm-sprd, with the modulus fixed at 0xff as the vendor kernel does it
 * (drivers/video/backlight/sprd_pwm_bl.c). Disabling stops the counter
 * with the output low; so does a duty cycle of 0, which is the vendor's
 * backlight-off state (pwm-backlight keeps the PWM enabled at duty 0
 * when there is no enable GPIO).
 *
 * Mainline pwm-sprd drives the later UMS512 block (separate MOD/DUTY and
 * ENABLE registers) and does not fit. Until there is a clock driver, the
 * channel gates (AON APB EB0 bits 4..7) and clock selects (bit 0 of one
 * register per channel, 1 = 26 MHz) are set here.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define SC8830_PWM_NUM		4
#define SC8830_PWM_STRIDE	0x20
#define PWM_PRESCALE		0x00
#define PWM_PRESCALE_EN		BIT(8)
#define PWM_PRESCALE_MASK	GENMASK(7, 0)
#define PWM_CNT			0x04
#define PWM_CNT_DUTY_SHIFT	8
#define PWM_MOD			0xff
#define PWM_PAT_LOW		0x0c
#define PWM_PAT_HIGH		0x10

#define AON_EB0_PWM(n)		BIT(4 + (n))
#define CLK_SEL_26M		BIT(0)

struct sc8830_pwm {
	void __iomem *base;
	void __iomem *aon_eb;	/* shared, only touched in request */
	void __iomem *clk_sel;
	unsigned long rate;
};

static void __iomem *sc8830_pwm_ch(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct sc8830_pwm *sp = pwmchip_get_drvdata(chip);

	return sp->base + pwm->hwpwm * SC8830_PWM_STRIDE;
}

static int sc8830_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct sc8830_pwm *sp = pwmchip_get_drvdata(chip);
	void __iomem *sel;

	if (sp->aon_eb)
		writel(readl(sp->aon_eb) | AON_EB0_PWM(pwm->hwpwm), sp->aon_eb);
	if (sp->clk_sel) {
		sel = sp->clk_sel + 4 * pwm->hwpwm;
		writel(readl(sel) | CLK_SEL_26M, sel);
	}
	return 0;
}

static int sc8830_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct sc8830_pwm *sp = pwmchip_get_drvdata(chip);
	void __iomem *ch = sc8830_pwm_ch(chip, pwm);
	u64 prescale, period, duty;

	if (state->polarity != PWM_POLARITY_NORMAL)
		return -EINVAL;

	if (!state->enabled) {
		writel(0, ch + PWM_PRESCALE);
		return 0;
	}

	/* round the period down to a prescaler step */
	prescale = mul_u64_u64_div_u64(state->period, sp->rate,
				       (u64)NSEC_PER_SEC * PWM_MOD);
	if (!prescale)
		return -EINVAL;
	prescale = min_t(u64, prescale, PWM_PRESCALE_MASK + 1);
	period = div_u64(prescale * PWM_MOD * NSEC_PER_SEC, sp->rate);
	duty = mul_u64_u64_div_u64(min(state->duty_cycle, period),
				   PWM_MOD, period);

	writel(duty << PWM_CNT_DUTY_SHIFT | PWM_MOD, ch + PWM_CNT);
	writel(0xffff, ch + PWM_PAT_LOW);
	writel(0xffff, ch + PWM_PAT_HIGH);
	writel(duty ? PWM_PRESCALE_EN | (prescale - 1) : prescale - 1,
	       ch + PWM_PRESCALE);
	return 0;
}

static int sc8830_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct sc8830_pwm *sp = pwmchip_get_drvdata(chip);
	void __iomem *ch = sc8830_pwm_ch(chip, pwm);
	u32 pre = readl(ch + PWM_PRESCALE), cnt = readl(ch + PWM_CNT);
	u64 step = ((pre & PWM_PRESCALE_MASK) + 1) * (u64)NSEC_PER_SEC;

	state->enabled = !!(pre & PWM_PRESCALE_EN);
	state->polarity = PWM_POLARITY_NORMAL;
	state->period = DIV_ROUND_UP_ULL(step * (cnt & 0xff), sp->rate);
	state->duty_cycle = DIV_ROUND_UP_ULL(step * (cnt >> PWM_CNT_DUTY_SHIFT & 0xff),
					     sp->rate);
	return 0;
}

static const struct pwm_ops sc8830_pwm_ops = {
	.request = sc8830_pwm_request,
	.apply = sc8830_pwm_apply,
	.get_state = sc8830_pwm_get_state,
};

static void __iomem *sc8830_pwm_shared(struct platform_device *pdev,
				       const char *name)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return NULL;
	/* registers shared with other blocks: no request_mem_region */
	return devm_ioremap(&pdev->dev, res->start, resource_size(res));
}

static int sc8830_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwm_chip *chip;
	struct sc8830_pwm *sp;
	struct clk *clk;

	chip = devm_pwmchip_alloc(dev, SC8830_PWM_NUM, sizeof(*sp));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	sp = pwmchip_get_drvdata(chip);

	sp->base = devm_platform_ioremap_resource_byname(pdev, "pwm");
	if (IS_ERR(sp->base))
		return PTR_ERR(sp->base);
	sp->aon_eb = sc8830_pwm_shared(pdev, "aon-apb");
	sp->clk_sel = sc8830_pwm_shared(pdev, "clk-sel");

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "no clock\n");
	sp->rate = clk_get_rate(clk);
	if (!sp->rate)
		return dev_err_probe(dev, -EINVAL, "clock without a rate\n");

	chip->ops = &sc8830_pwm_ops;
	return devm_pwmchip_add(dev, chip);
}

static const struct of_device_id sc8830_pwm_of_match[] = {
	{ .compatible = "sprd,sc8830-pwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc8830_pwm_of_match);

static struct platform_driver sc8830_pwm_driver = {
	.probe = sc8830_pwm_probe,
	.driver = {
		.name = "sc8830-pwm",
		.of_match_table = sc8830_pwm_of_match,
	},
};
module_platform_driver(sc8830_pwm_driver);

MODULE_DESCRIPTION("Spreadtrum SC8830/SC7730 PWM driver");
MODULE_LICENSE("GPL");
