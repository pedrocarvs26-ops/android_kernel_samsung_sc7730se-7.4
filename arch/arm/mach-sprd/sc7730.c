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
 * While CPU0 waits, the DDR controller (Synopsys uMCTL2) may put the DRAM
 * into self-refresh after its idle timeout (PWRTMG, set by the SPL) and
 * takes it out on the next access, as in the vendor's suspend: only PWRCTL
 * changes, no code runs from IRAM. Measured with the panel off: 89 -> 88
 * mA; the controller already powers the DRAM down when idle. The vendor's
 * light-sleep bits (MCU_PAUSE SYS/LIGHT_SLEEP with the AON wake mux) gave
 * nothing measurable while peripheral clocks run, and the core clock
 * auto gate stops the CPU's view of the system counter (lost time). The
 * vendor's deep sleep (peripheral gating, CA7 power-down, DDR PHY
 * retention, resume from IRAM) is not there yet.
 *
 * The CPU OPP table goes beyond the vendor's 1300 MHz (overclocking steps
 * checked on a single unit), so every new cpufreq policy gets a maximum
 * of 1300 MHz before its governor starts. User space can move it through
 * /sys/module/sc7730/parameters/cpu_max_khz.
 */

#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_qos.h>
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
#define UMCTL2_PHYS		0x30000000
#define UMCTL2_PWRCTL		0x30
#define PWRCTL_SELFREF_EN	BIT(0)
#define PWRCTL_POWERDOWN_EN	BIT(1)
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
static void __iomem *sc7730_umctl2;

static bool ddr_selfref = true;
module_param(ddr_selfref, bool, 0644);
MODULE_PARM_DESC(ddr_selfref, "DDR self-refresh while suspended");

static int sc7730_suspend_enter(suspend_state_t state)
{
	void __iomem *pwrctl = sc7730_umctl2 + UMCTL2_PWRCTL;
	bool selfref = sc7730_umctl2 && READ_ONCE(ddr_selfref);
	u32 old = 0;

	/* the vendor's suspend: self-refresh instead of power-down */
	if (selfref) {
		old = readl(pwrctl);
		writel((old & ~PWRCTL_POWERDOWN_EN) | PWRCTL_SELFREF_EN, pwrctl);
	}
	cpu_do_idle();
	if (selfref)
		writel(old, pwrctl);
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
	sc7730_umctl2 = ioremap(UMCTL2_PHYS, 0x100);
	suspend_set_ops(&sc7730_suspend_ops);
}
#else
#define sc7730_init_late	NULL
#endif

#ifdef CONFIG_CPU_FREQ
static unsigned int cpu_max_khz = 1300000;
static struct freq_qos_request cpu_max_req;
static struct cpufreq_policy *cpu_max_policy;
static DEFINE_MUTEX(cpu_max_lock);

static int sc7730_cpu_max_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	mutex_lock(&cpu_max_lock);
	ret = param_set_uint(val, kp);
	if (!ret && cpu_max_policy)
		ret = freq_qos_update_request(&cpu_max_req, cpu_max_khz);
	mutex_unlock(&cpu_max_lock);
	return ret < 0 ? ret : 0;
}

static const struct kernel_param_ops sc7730_cpu_max_ops = {
	.set = sc7730_cpu_max_set,
	.get = param_get_uint,
};
module_param_cb(cpu_max_khz, &sc7730_cpu_max_ops, &cpu_max_khz, 0644);
MODULE_PARM_DESC(cpu_max_khz, "CPU frequency limit in kHz (vendor maximum 1300000)");

static int sc7730_cpufreq_notify(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;

	mutex_lock(&cpu_max_lock);
	if (event == CPUFREQ_CREATE_POLICY && !cpu_max_policy) {
		if (freq_qos_add_request(&policy->constraints, &cpu_max_req,
					 FREQ_QOS_MAX, cpu_max_khz) >= 0)
			cpu_max_policy = policy;
	} else if (event == CPUFREQ_REMOVE_POLICY && policy == cpu_max_policy) {
		freq_qos_remove_request(&cpu_max_req);
		cpu_max_policy = NULL;
	}
	mutex_unlock(&cpu_max_lock);
	return NOTIFY_OK;
}

static struct notifier_block sc7730_cpufreq_nb = {
	.notifier_call = sc7730_cpufreq_notify,
};

/* before cpufreq-dt probes at device_initcall */
static int __init sc7730_cpufreq_limit_init(void)
{
	if (!of_machine_is_compatible("sprd,sc7730"))
		return 0;
	return cpufreq_register_notifier(&sc7730_cpufreq_nb,
					 CPUFREQ_POLICY_NOTIFIER);
}
arch_initcall(sc7730_cpufreq_limit_init);
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
