// SPDX-License-Identifier: GPL-2.0
/*
 * The CLK_AUX0 output of the SC7730, on the gtelwifi the 32 kHz low power
 * clock (LPO) of the BCM43430 WLAN chip. A gate (AON APB EB1 bit 2) on the
 * external 32 kHz clock, with source and divider in AON_CGM_CFG set to
 * 32 kHz / 1 as in the vendor board file. The bootloader leaves the gate
 * off and already muxes the pad to CLK_AUX0.
 *
 * Bring-up glue until there is a clock driver for this SoC.
 */

#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define AON_APB_EB1		0x0004
#define AON_EB1_AUX0_BIT	2
#define AON_CGM_CFG		0x3020
#define AON_CGM_AUX0_SEL	GENMASK(3, 0)	/* 0: 32 kHz */
#define AON_CGM_AUX0_DIV	GENMASK(19, 16)	/* 0: divide by 1 */

static DEFINE_SPINLOCK(sc7730_aux0_lock);

static int sc7730_aux0_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem *aon;
	const char *parent;
	struct clk_hw *hw;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	/* shared with other blocks, so no request_mem_region */
	aon = devm_ioremap(dev, res->start, resource_size(res));
	if (!aon)
		return -ENOMEM;

	parent = of_clk_get_parent_name(dev->of_node, 0);
	if (!parent)
		return dev_err_probe(dev, -EINVAL, "no parent clock\n");

	writel(readl(aon + AON_CGM_CFG) &
	       ~(AON_CGM_AUX0_SEL | AON_CGM_AUX0_DIV), aon + AON_CGM_CFG);

	hw = devm_clk_hw_register_gate(dev, "aux0", parent, 0,
				       aon + AON_APB_EB1, AON_EB1_AUX0_BIT, 0,
				       &sc7730_aux0_lock);
	if (IS_ERR(hw))
		return PTR_ERR(hw);

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, hw);
}

static const struct of_device_id sc7730_aux0_of_match[] = {
	{ .compatible = "sprd,sc7730-aux0-clk" },
	{ }
};

static struct platform_driver sc7730_aux0_driver = {
	.probe = sc7730_aux0_probe,
	.driver = {
		.name = "sc7730-aux0-clk",
		.of_match_table = sc7730_aux0_of_match,
	},
};
builtin_platform_driver(sc7730_aux0_driver);
