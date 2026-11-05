// SPDX-License-Identifier: GPL-2.0
/*
 * Power domains of the SC7730 PMU as generic PM domains, so that drivers
 * get them through "power-domains" and runtime PM: GPU_TOP (Mali-400 MP2)
 * and MM_TOP (camera, video).
 *
 * Each domain has a PD_*_CFG register in the PMU (0x402b0000): bit 25
 * forces it off, bit 24 would let the PMU switch it off on its own in
 * deep sleep (left off), bits 23:0 are power-up delays. Its state is a
 * nibble in PWR_STATUS0_DBG, 0 = on, 7 = off. Behind the domain sits an
 * AXI/AHB enable bit in AON_APB EB0 (0x402e0000). Sequence as in the vendor
 * Mali glue (drivers/gpu/mali400/r4p1/platform/sc8830/mali_platform.c):
 * delays 1/1/1, clear FORCE_SHUTDOWN, wait for state 0, 100 us, enable.
 *
 * The GPU's core clock mux lives inside the domain (GPU_APB CLK_CTRL at
 * 0x60100004, SEL bits 2:0, DIV bits 5:4) and loses its setting with it,
 * so it is programmed here on every power-up (the reset value is 256 MHz,
 * TDPLL / 3). The rate comes from the gpu_khz parameter:
 *
 *   384000  SEL 5, TDPLL (768 MHz) / 2, always running
 *   460800  SEL 6, WPLL (921.6 MHz) / 2, the vendor's maximum for this SoC
 *   512000  SEL 4, CPLL / 2 (default); the SPL programs CPLL to 1024 MHz,
 *           not the 624 MHz the vendor DT names (clk_312m) assume
 *
 * WPLL and CPLL belong to the modem and are powered down unless somebody
 * votes for them (ANALOG_STATUS PD bits set; selecting one of them then
 * hung the GPU bus, 01.11.). The AP votes through bit 0 of *PLL_REL_CFG in
 * the PMU while the GPU domain is on and drops the vote with it, as the
 * vendor Mali glue does by preparing all its source clocks. Measured with
 * a GLES 2 fill-rate test (tools/lab/gpubench.c): 21.5 / 25.6 / 28.5 fps,
 * 512 MHz stable over 3 minutes at the fixed vddcore of the SPL (05.11.).
 * If the PLL does not come up, the GPU stays at 384 MHz. A new value
 * takes effect at the next power-up of the domain. A clock driver would
 * read the mux while the domain is off.
 *
 * Bring-up glue until there is a power domain driver for this SoC.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/spinlock.h>

#define PD_FORCE_SHUTDOWN	BIT(25)
#define PD_AUTO_SHUTDOWN	BIT(24)
#define PD_DELAYS_MASK		GENMASK(23, 0)
#define PD_DELAYS		0x010101
#define PWR_STATUS0_DBG		0xbc
#define AON_APB_EB0		0x00

#define GPU_APB_PHYS		0x60100000
#define GPU_APB_CLK_CTRL	0x04
#define GPU_CLK_SEL		GENMASK(2, 0)
#define GPU_CLK_DIV		GENMASK(5, 4)
#define GPU_CLK_SEL_384M	5

#define ANALOG_STATUS_PHYS	0x402e3090
#define WPLL_REL_CFG		0xa0
#define CPLL_REL_CFG		0xa4
#define PLL_AP_SEL		BIT(0)

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "sc7730."

static unsigned int gpu_khz = 512000;
module_param(gpu_khz, uint, 0644);
MODULE_PARM_DESC(gpu_khz, "GPU clock: 384000, 460800 or 512000 (next power-up)");

/* GPU clock sources besides TDPLL / 2: modem PLL, its vote and PD flag */
static const struct sc7730_gpu_src {
	unsigned int khz;
	u32 sel;
	u32 rel_cfg;
	u32 pd_bit;		/* in ANALOG_STATUS, 1 = powered down */
} sc7730_gpu_srcs[] = {
	{ 460800, 6, WPLL_REL_CFG, BIT(15) },
	{ 512000, 4, CPLL_REL_CFG, BIT(17) },
};

enum { SC7730_PD_GPU, SC7730_PD_MM, SC7730_NR_PDS };

struct sc7730_pd_data {
	const char *name;
	u32 cfg;		/* PD_*_CFG offset in the PMU */
	u32 state_shift;	/* nibble in PWR_STATUS0_DBG */
	u32 eb;			/* enable bit in AON_APB EB0 */
};

static const struct sc7730_pd_data sc7730_pd_data[SC7730_NR_PDS] = {
	[SC7730_PD_GPU] = { "gpu", 0x20, 24, BIT(27) },
	[SC7730_PD_MM]  = { "mm",  0x1c, 28, BIT(25) },
};

struct sc7730_pd {
	struct generic_pm_domain genpd;
	const struct sc7730_pd_data *data;
	struct sc7730_pds *pds;
};

struct sc7730_pds {
	void __iomem *pmu, *aon, *gpu_apb, *analog_status;
	const struct sc7730_gpu_src *gpu_src;	/* voted while the domain is on */
	struct device *dev;
	struct sc7730_pd pd[SC7730_NR_PDS];
	struct generic_pm_domain *domains[SC7730_NR_PDS];
	struct genpd_onecell_data onecell;
};

/* EB0 is also written directly by other bring-up glue */
static DEFINE_SPINLOCK(sc7730_pd_lock);

static void sc7730_pd_rmw(void __iomem *reg, u32 clr, u32 set)
{
	unsigned long flags;

	spin_lock_irqsave(&sc7730_pd_lock, flags);
	writel((readl(reg) & ~clr) | set, reg);
	spin_unlock_irqrestore(&sc7730_pd_lock, flags);
}

/* Vote for the PLL behind gpu_khz and return the mux setting to use */
static u32 sc7730_gpu_clk_sel(struct sc7730_pds *pds)
{
	const struct sc7730_gpu_src *src = NULL;
	unsigned int khz = READ_ONCE(gpu_khz);
	u32 v;
	int i;

	for (i = 0; i < ARRAY_SIZE(sc7730_gpu_srcs); i++)
		if (sc7730_gpu_srcs[i].khz == khz)
			src = &sc7730_gpu_srcs[i];
	if (!src || !pds->analog_status)
		return GPU_CLK_SEL_384M;

	sc7730_pd_rmw(pds->pmu + src->rel_cfg, 0, PLL_AP_SEL);
	if (readl_poll_timeout_atomic(pds->analog_status, v, !(v & src->pd_bit),
				      10, 2000)) {
		sc7730_pd_rmw(pds->pmu + src->rel_cfg, PLL_AP_SEL, 0);
		dev_warn_once(pds->dev, "PLL for %u kHz does not start, GPU at 384 MHz\n",
			      khz);
		return GPU_CLK_SEL_384M;
	}
	pds->gpu_src = src;
	return src->sel;
}

static int sc7730_pd_power_on(struct generic_pm_domain *genpd)
{
	struct sc7730_pd *pd = container_of(genpd, struct sc7730_pd, genpd);
	struct sc7730_pds *pds = pd->pds;
	const struct sc7730_pd_data *d = pd->data;
	u32 v;
	int ret;

	sc7730_pd_rmw(pds->pmu + d->cfg,
		      PD_FORCE_SHUTDOWN | PD_AUTO_SHUTDOWN | PD_DELAYS_MASK,
		      PD_DELAYS);
	ret = readl_poll_timeout_atomic(pds->pmu + PWR_STATUS0_DBG, v,
					!((v >> d->state_shift) & 0xf),
					50, 100000);
	if (ret) {
		dev_err(pds->dev, "%s domain does not power up (status %08x)\n",
			d->name, v);
		return ret;
	}
	udelay(100);
	sc7730_pd_rmw(pds->aon + AON_APB_EB0, 0, d->eb);

	if (d == &sc7730_pd_data[SC7730_PD_GPU] && pds->gpu_apb)
		writel(FIELD_PREP(GPU_CLK_SEL, sc7730_gpu_clk_sel(pds)) |
		       FIELD_PREP(GPU_CLK_DIV, 0),
		       pds->gpu_apb + GPU_APB_CLK_CTRL);
	return 0;
}

static int sc7730_pd_power_off(struct generic_pm_domain *genpd)
{
	struct sc7730_pd *pd = container_of(genpd, struct sc7730_pd, genpd);
	struct sc7730_pds *pds = pd->pds;
	const struct sc7730_pd_data *d = pd->data;

	sc7730_pd_rmw(pds->aon + AON_APB_EB0, d->eb, 0);
	sc7730_pd_rmw(pds->pmu + d->cfg, 0, PD_FORCE_SHUTDOWN);

	/* the GPU is the only AP user of the modem PLLs */
	if (d == &sc7730_pd_data[SC7730_PD_GPU] && pds->gpu_src) {
		sc7730_pd_rmw(pds->pmu + pds->gpu_src->rel_cfg, PLL_AP_SEL, 0);
		pds->gpu_src = NULL;
	}
	return 0;
}

static int sc7730_pds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc7730_pds *pds;
	struct resource *res;
	int i, ret;

	pds = devm_kzalloc(dev, sizeof(*pds), GFP_KERNEL);
	if (!pds)
		return -ENOMEM;
	pds->dev = dev;

	/* both register blocks are shared with other glue: no request */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pmu");
	if (res)
		pds->pmu = devm_ioremap(dev, res->start, resource_size(res));
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "aon-apb");
	if (res)
		pds->aon = devm_ioremap(dev, res->start, resource_size(res));
	if (!pds->pmu || !pds->aon)
		return -ENOMEM;
	/* only touched while the GPU domain is on */
	pds->gpu_apb = devm_ioremap(dev, GPU_APB_PHYS, 0x10);
	pds->analog_status = devm_ioremap(dev, ANALOG_STATUS_PHYS, 4);

	for (i = 0; i < SC7730_NR_PDS; i++) {
		struct sc7730_pd *pd = &pds->pd[i];
		u32 st = readl(pds->pmu + PWR_STATUS0_DBG);
		bool off;

		pd->data = &sc7730_pd_data[i];
		pd->pds = pds;
		pd->genpd.name = pd->data->name;
		pd->genpd.power_on = sc7730_pd_power_on;
		pd->genpd.power_off = sc7730_pd_power_off;
		/*
		 * Off unless the bootloader left it on; genpd switches unused
		 * domains off late in boot.
		 */
		off = (st >> pd->data->state_shift) & 0xf;
		ret = pm_genpd_init(&pd->genpd, NULL, off);
		if (ret)
			return ret;
		pds->domains[i] = &pd->genpd;
		dev_info(dev, "%s domain %s at boot\n", pd->data->name,
			 off ? "off" : "on");
	}

	pds->onecell.domains = pds->domains;
	pds->onecell.num_domains = SC7730_NR_PDS;
	return of_genpd_add_provider_onecell(dev->of_node, &pds->onecell);
}

static const struct of_device_id sc7730_pds_of_match[] = {
	{ .compatible = "sprd,sc7730-pmu-domains" },
	{ }
};

static struct platform_driver sc7730_pds_driver = {
	.probe = sc7730_pds_probe,
	.driver = {
		.name = "sc7730-pmu-domains",
		.of_match_table = sc7730_pds_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(sc7730_pds_driver);
