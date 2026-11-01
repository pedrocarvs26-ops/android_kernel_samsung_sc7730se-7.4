// SPDX-License-Identifier: GPL-2.0
/*
 * SDHCI glue for the Spreadtrum SC7730/SC8830 ("shark") SD/SDIO/eMMC hosts.
 *
 * The controllers are plain SDHCI 3.0. The vendor driver
 * (drivers/mmc/host/sprd-sdhci-fix-of.c in the 3.10 tree) only adds the
 * quirks below and takes the base clock from the device tree, because the
 * capabilities register does not report it.
 *
 * Until this SoC has a clock driver, the base clock is read back from the
 * AP clock mux the bootloader programmed ("clk-cfg", bits 1:0 select one of
 * "sprd,clk-parent-rates"), so the card is never clocked faster than the
 * divider math assumes. Gate, reset, pinmux and card power are left as the
 * bootloader set them up.
 */

#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "sdhci-pltfm.h"

#define SC7730_CLK_SEL_MASK	0x3

struct sc7730_sdhci {
	unsigned int base_clk;
};

static unsigned int sc7730_sdhci_get_max_clock(struct sdhci_host *host)
{
	struct sc7730_sdhci *priv = sdhci_pltfm_priv(sdhci_priv(host));

	return priv->base_clk;
}

static const struct sdhci_ops sc7730_sdhci_ops = {
	.set_clock		= sdhci_set_clock,
	.get_max_clock		= sc7730_sdhci_get_max_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.reset			= sdhci_reset,
	.set_uhs_signaling	= sdhci_set_uhs_signaling,
};

static const struct sdhci_pltfm_data sc7730_sdhci_pdata = {
	.ops = &sc7730_sdhci_ops,
	.quirks = SDHCI_QUIRK_CLOCK_BEFORE_RESET |
		  SDHCI_QUIRK_BROKEN_CARD_DETECTION |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_NO_HISPD_BIT,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
};

static int sc7730_sdhci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc7730_sdhci *priv;
	struct sdhci_host *host;
	void __iomem *clk_cfg;
	u32 rates[4], sel;
	int ret;

	host = sdhci_pltfm_init(pdev, &sc7730_sdhci_pdata, sizeof(*priv));
	if (IS_ERR(host))
		return PTR_ERR(host);
	priv = sdhci_pltfm_priv(sdhci_priv(host));

	clk_cfg = devm_platform_ioremap_resource_byname(pdev, "clk-cfg");
	if (IS_ERR(clk_cfg))
		return PTR_ERR(clk_cfg);
	ret = of_property_read_u32_array(dev->of_node, "sprd,clk-parent-rates",
					 rates, ARRAY_SIZE(rates));
	if (ret)
		return dev_err_probe(dev, ret, "missing sprd,clk-parent-rates\n");

	sel = readl(clk_cfg) & SC7730_CLK_SEL_MASK;
	priv->base_clk = rates[sel];
	dev_info(dev, "base clock %u Hz (mux %u), caps %08x %08x\n",
		 priv->base_clk, sel, sdhci_readl(host, SDHCI_CAPABILITIES),
		 sdhci_readl(host, SDHCI_CAPABILITIES_1));

	ret = mmc_of_parse(host->mmc);
	if (ret)
		return ret;
	sdhci_get_of_property(pdev);

	return sdhci_add_host(host);
}

static const struct of_device_id sc7730_sdhci_of_match[] = {
	{ .compatible = "sprd,sc7730-sdhci" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc7730_sdhci_of_match);

static struct platform_driver sc7730_sdhci_driver = {
	.probe = sc7730_sdhci_probe,
	.remove = sdhci_pltfm_remove,
	.driver = {
		.name = "sdhci-sc7730",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = sc7730_sdhci_of_match,
	},
};
module_platform_driver(sc7730_sdhci_driver);

MODULE_DESCRIPTION("SDHCI glue for Spreadtrum SC7730/SC8830");
MODULE_LICENSE("GPL");
