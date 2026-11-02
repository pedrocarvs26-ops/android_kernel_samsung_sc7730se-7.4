// SPDX-License-Identifier: GPL-2.0
/*
 * Cortex-A7 cluster clock of the SC7730 for CPU frequency scaling.
 *
 * CA7_CKG_CFG (AP AHB 0x20d03000) selects the core source (bits 2:0,
 * 3 = TDPLL 768 MHz, 6 = MPLL) and a core divider (bits 6:4, kept at 0);
 * the AXI divider (bits 10:8) stays at the SPL's core/3 up to 1.5 GHz and
 * goes to core/4 above, so AXI never runs faster than the 500 MHz the SPL
 * itself uses; the debug divider (core/7) is left alone. MPLL_CFG1 (AON
 * APB 0x402e3070) sets
 * MPLL = 26 MHz * (NINT + KINT / 2^20) with DIV_S set by the SPL.
 *
 * As in the vendor kernel (drivers/clk/clk-sc.c), a rate change parks the
 * core on TDPLL, reprograms MPLL, waits 50 us per 100 MHz of change (there
 * is no lock bit) and switches back. 768 MHz runs from TDPLL directly.
 * MPLL feeds nothing else we use.
 */

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define CA7_CKG_SEL		GENMASK(2, 0)
#define  CKG_SEL_26M		0
#define  CKG_SEL_TDPLL		3
#define  CKG_SEL_MPLL		6
#define CA7_CKG_DIV		GENMASK(6, 4)
#define CA7_AXI_DIV		GENMASK(10, 8)
#define  AXI_DIV_3		2
#define  AXI_DIV_4		3

#define MPLL_NINT		GENMASK(5, 0)
#define MPLL_SDM_EN		BIT(6)
#define MPLL_DIV_S		BIT(10)
#define MPLL_KINT		GENMASK(31, 12)

#define REF_HZ			26000000UL
#define TDPLL_HZ		768000000UL
#define MPLL_MIN_HZ		(REF_HZ * 30)	/* 780 MHz */
#define MPLL_MAX_HZ		(REF_HZ * 63)	/* NINT is 6 bits: 1638 MHz */
#define AXI_DIV3_MAX_HZ		1500000000UL

struct sc7730_cpu_clk {
	struct clk_hw hw;
	void __iomem *ckg;
	void __iomem *mpll;
};

#define to_cpu_clk(_hw) container_of(_hw, struct sc7730_cpu_clk, hw)

static unsigned long sc7730_mpll_rate(u32 cfg1)
{
	u64 rate = (u64)REF_HZ * FIELD_GET(MPLL_NINT, cfg1);

	if (cfg1 & MPLL_SDM_EN)
		rate += div_u64((u64)REF_HZ * FIELD_GET(MPLL_KINT, cfg1),
				1 << 20);
	return rate;
}

/* to the kHz: KINT leaves odd Hz that cpufreq would not find in the table */
static unsigned long sc7730_cpu_clk_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct sc7730_cpu_clk *cc = to_cpu_clk(hw);
	u32 ckg = readl(cc->ckg);
	unsigned long rate;

	switch (FIELD_GET(CA7_CKG_SEL, ckg)) {
	case CKG_SEL_MPLL:
		rate = sc7730_mpll_rate(readl(cc->mpll));
		break;
	case CKG_SEL_TDPLL:
		rate = TDPLL_HZ;
		break;
	case CKG_SEL_26M:
		rate = REF_HZ;
		break;
	default:
		return 0;
	}
	rate /= FIELD_GET(CA7_CKG_DIV, ckg) + 1;
	return DIV_ROUND_CLOSEST(rate, 1000) * 1000;
}

static int sc7730_cpu_clk_determine_rate(struct clk_hw *hw,
					 struct clk_rate_request *req)
{
	if (req->rate <= TDPLL_HZ)
		req->rate = TDPLL_HZ;
	else
		req->rate = clamp(req->rate, MPLL_MIN_HZ, MPLL_MAX_HZ);
	return 0;
}

static void sc7730_cpu_clk_select(struct sc7730_cpu_clk *cc, u32 sel,
				  u32 axi_div)
{
	u32 ckg = readl(cc->ckg);

	ckg &= ~(CA7_CKG_SEL | CA7_CKG_DIV | CA7_AXI_DIV);
	writel(ckg | FIELD_PREP(CA7_CKG_SEL, sel) |
	       FIELD_PREP(CA7_AXI_DIV, axi_div), cc->ckg);
}

static int sc7730_cpu_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct sc7730_cpu_clk *cc = to_cpu_clk(hw);
	u32 axi_div = rate > AXI_DIV3_MAX_HZ ? AXI_DIV_4 : AXI_DIV_3;
	u32 cfg1 = readl(cc->mpll), new;
	unsigned long old_hz, nint, kint, delta;

	sc7730_cpu_clk_select(cc, CKG_SEL_TDPLL, AXI_DIV_3);
	if (rate == TDPLL_HZ)
		return 0;

	nint = rate / REF_HZ;
	kint = DIV_ROUND_CLOSEST_ULL((u64)(rate - nint * REF_HZ) << 20,
				     REF_HZ);
	new = cfg1 & ~(MPLL_NINT | MPLL_KINT | MPLL_SDM_EN);
	new |= FIELD_PREP(MPLL_NINT, nint) | MPLL_DIV_S;
	if (kint)
		new |= FIELD_PREP(MPLL_KINT, kint) | MPLL_SDM_EN;

	if (new != cfg1) {
		old_hz = sc7730_mpll_rate(cfg1);
		writel(new, cc->mpll);
		delta = abs_diff(old_hz, sc7730_mpll_rate(new)) / 1000000;
		/* vendor: 50 us per 100 MHz; never less than 100 us */
		fsleep(max(100UL, delta / 2));
	}

	sc7730_cpu_clk_select(cc, CKG_SEL_MPLL, axi_div);
	return 0;
}

static const struct clk_ops sc7730_cpu_clk_ops = {
	.recalc_rate = sc7730_cpu_clk_recalc_rate,
	.determine_rate = sc7730_cpu_clk_determine_rate,
	.set_rate = sc7730_cpu_clk_set_rate,
};

static int sc7730_cpu_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_init_data init = {};
	struct sc7730_cpu_clk *cc;
	struct resource *res;
	u32 cfg1;
	int ret;

	cc = devm_kzalloc(dev, sizeof(*cc), GFP_KERNEL);
	if (!cc)
		return -ENOMEM;

	/* both registers sit in blocks shared with other drivers */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ckg");
	if (res)
		cc->ckg = devm_ioremap(dev, res->start, resource_size(res));
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mpll");
	if (res)
		cc->mpll = devm_ioremap(dev, res->start, resource_size(res));
	if (!cc->ckg || !cc->mpll)
		return dev_err_probe(dev, -EINVAL, "missing ckg/mpll\n");

	cfg1 = readl(cc->mpll);
	if (!(cfg1 & MPLL_DIV_S))
		return dev_err_probe(dev, -ENODEV,
				     "MPLL not in NINT/KINT mode (%#x)\n", cfg1);

	init.name = "cpu";
	init.ops = &sc7730_cpu_clk_ops;
	init.flags = CLK_IS_CRITICAL | CLK_GET_RATE_NOCACHE;
	cc->hw.init = &init;

	ret = devm_clk_hw_register(dev, &cc->hw);
	if (ret)
		return ret;

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &cc->hw);
}

static const struct of_device_id sc7730_cpu_clk_of_match[] = {
	{ .compatible = "sprd,sc7730-cpu-clk" },
	{ }
};

static struct platform_driver sc7730_cpu_clk_driver = {
	.probe = sc7730_cpu_clk_probe,
	.driver = {
		.name = "sc7730-cpu-clk",
		.of_match_table = sc7730_cpu_clk_of_match,
	},
};
builtin_platform_driver(sc7730_cpu_clk_driver);
