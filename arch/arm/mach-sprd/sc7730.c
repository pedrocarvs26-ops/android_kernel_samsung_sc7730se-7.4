// SPDX-License-Identifier: GPL-2.0
/*
 * Spreadtrum SC7730: start the system counter before the timers probe.
 *
 * The ARM architected timers of the Cortex-A7 cluster count the CoreSight
 * timestamp generator TS1 (TS0 also runs, but does not reach the cores),
 * which is off after reset: AON_APB EB0 bit 29 gates it, AON_CLK
 * CA7_TS_CFG bits 1:0 select its clock (1 = 26 MHz), CNTCR bit 0 starts
 * it, and the timestamp ports have a low-power handshake that has to
 * request them active (AON_APB TS_CFG CSYSREQ bits 8/10/12). Neither
 * bootloader stage touches any of this; the vendor kernel never used the
 * architected timer at all.
 *
 * The bootloader also leaves the kernel in the Secure world (NSACR reads 0
 * with VFP working), so CNTFRQ is ours to set, on every CPU.
 *
 * With the SoC's automatic core clock gating (AP_AHB AP_SYS_AUTO_SLEEP_CFG
 * bit 0, on after reset) the clock of a core in WFI stops, and with it the
 * comparators of its architected timers: neither the PPI nor the SPI copy
 * fires, and an idle CPU only wakes for some other interrupt (100 sleeps
 * of 10 ms took 124 s). Keep the core clocks running in WFI; the vendor
 * kernel does the same on some scx35 revisions.
 *
 * Suspend to "mem" is the vendor's standby for now: the PM core takes
 * CPU1-3 offline, which powers them down (platsmp.c), and CPU0 waits in
 * WFI for a wakeup interrupt. DDR, the GIC and the counter stay powered.
 * The vendor's deep sleep (DDR self-refresh, CA7 power-down, resume from
 * IRAM) is not there yet.
 */

#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/printk.h>
#include <linux/suspend.h>

#include <asm/mach/arch.h>
#include <asm/proc-fns.h>

#include "sc7730.h"

#define AON_APB_PHYS		0x402e0000
#define AON_APB_EB0		0x00
#define AON_EB0_CA7_TS1		BIT(29)
#define AON_APB_TS_CFG		0x28
#define TS_CFG_CSYSREQ		(BIT(8) | BIT(10) | BIT(12))
#define AON_CLK_CA7_TS_CFG	0x402d004c
#define CA7_TS_SEL_26M		1
#define CA7_TS1_PHYS		0x40410000
#define TS_CNTCR_EN		BIT(0)
#define AP_AHB_PHYS		0x20d00000
#define AP_SYS_AUTO_SLEEP_CFG	0x10
#define CA7_CORE_AUTO_GATE_EN	BIT(0)

void sc7730_set_cntfrq(void)
{
	asm volatile("mcr p15, 0, %0, c14, c0, 0" : : "r" (SC7730_CNTFRQ));
	isb();
}

static void __init sc7730_counter_init(void)
{
	void __iomem *aon = ioremap(AON_APB_PHYS, 0x30);
	void __iomem *clk = ioremap(AON_CLK_CA7_TS_CFG, 4);
	void __iomem *ts1 = ioremap(CA7_TS1_PHYS, 0x10);
	void __iomem *ahb = ioremap(AP_AHB_PHYS, 0x20);

	if (ahb) {
		writel_relaxed(readl_relaxed(ahb + AP_SYS_AUTO_SLEEP_CFG) &
			       ~CA7_CORE_AUTO_GATE_EN, ahb + AP_SYS_AUTO_SLEEP_CFG);
		iounmap(ahb);
	}

	if (aon && clk && ts1) {
		writel_relaxed(readl_relaxed(aon + AON_APB_EB0) | AON_EB0_CA7_TS1,
			       aon + AON_APB_EB0);
		writel_relaxed((readl_relaxed(clk) & ~3) | CA7_TS_SEL_26M, clk);
		writel_relaxed(readl_relaxed(aon + AON_APB_TS_CFG) | TS_CFG_CSYSREQ,
			       aon + AON_APB_TS_CFG);
		writel(readl_relaxed(ts1) | TS_CNTCR_EN, ts1);
		sc7730_set_cntfrq();
	} else {
		pr_err("sc7730: cannot map the system counter\n");
	}

	if (ts1)
		iounmap(ts1);
	if (clk)
		iounmap(clk);
	if (aon)
		iounmap(aon);
}

static void __init sc7730_init_time(void)
{
	sc7730_counter_init();
	of_clk_init(NULL);
	timer_probe();
}

#ifdef CONFIG_SUSPEND
static int sc7730_suspend_enter(suspend_state_t state)
{
	cpu_do_idle();
	return 0;
}

/*
 * WFI ends for any pending interrupt, also one that is not a wakeup
 * source (a touch controller losing power toggles its line on the way
 * down). Once the core has looked at it, after resuming as far as the
 * early phase, sleep again unless it was a wakeup. Handling interrupts
 * inside ->enter() instead, with timekeeping suspended, hung the resume.
 */
static bool sc7730_suspend_again(void)
{
	return !pm_wakeup_pending();
}

static const struct platform_suspend_ops sc7730_suspend_ops = {
	.valid		= suspend_valid_only_mem,
	.enter		= sc7730_suspend_enter,
	.suspend_again	= sc7730_suspend_again,
};

static void __init sc7730_init_late(void)
{
	suspend_set_ops(&sc7730_suspend_ops);
}
#else
#define sc7730_init_late	NULL
#endif

static const char *const sc7730_dt_compat[] __initconst = {
	"sprd,sc7730",
	NULL
};

DT_MACHINE_START(SPRD_SC7730, "Spreadtrum SC7730")
	.dt_compat	= sc7730_dt_compat,
	.init_time	= sc7730_init_time,
	.init_late	= sc7730_init_late,
MACHINE_END
