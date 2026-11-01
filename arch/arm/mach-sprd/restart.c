// SPDX-License-Identifier: GPL-2.0
/*
 * Restart the SC7730 through the SC2723 PMIC watchdog, the way the Samsung
 * vendor kernel does it (arch_reset() + sec-reboot.c + sec_debug.c):
 *
 *   - clear Samsung's upload magic; the vendor kernel arms it at boot and
 *     only clears it on a clean reboot
 *   - INFORM2 = "no low-power charging", INFORM3 = reboot reason; sboot's
 *     sprd_check_reboot_mode() takes (INFORM3 & 0xf) as the boot mode when
 *     INFORM3 & ~0xf == 0x12345670 (4 = recovery)
 *   - PMIC POR_RST_MONITOR = Spreadtrum boot mode, as the vendor does
 *     (sboot ignores it)
 *   - start the PMIC watchdog with 50 ms and wait for it
 *
 * The vendor header puts the magic at 0x50003000, which holds code on
 * this board. Disassembling SBOOT (loaded at 0x9f600000) shows sboot
 * reading the magic at 0xf80 and INFORM3 at 0xf90 in IRAM0 instead.
 * The IRAM is only touched when restarting, never at boot.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/reboot.h>
#include <linux/string.h>

#include "adi.h"

#define SEC_DEBUG_PHYS		0x00000f80	/* IRAM0, see above */
#define SEC_UPLOAD_MAGIC	0x00
#define SEC_INFORM2		0x0c
#define SEC_INFORM3		0x10
#define SEC_NO_LPM		0x12345678
#define SEC_MODE(m)		(0x12345670 | (m))
#define SEC_MODE_NONE		0
#define SEC_MODE_DOWNLOAD	1
#define SEC_MODE_RECOVERY	4

#define GLB_ARM_MODULE_EN	SC2723_GLB(0x000)
#define GLB_RTC_CLK_EN		SC2723_GLB(0x008)
#define GLB_POR_RST_MONITOR	SC2723_GLB(0x124)
#define GLB_WDG_EN		BIT(2)		/* in both enable registers */

#define HWRST_RECOVERY		0x20
#define HWRST_NORMAL		0x40
#define HWRST_SPECIAL		0x70

#define WDG_LOAD_LOW		SC2723_WDG(0x00)
#define WDG_LOAD_HIGH		SC2723_WDG(0x04)
#define WDG_CTRL		SC2723_WDG(0x08)
#define WDG_CNT_EN		BIT(1)
#define WDG_NEW_VER_EN		BIT(2)
#define WDG_RST_EN		BIT(3)
#define WDG_INT_RAW		SC2723_WDG(0x10)
#define WDG_LD_BUSY		BIT(4)
#define WDG_LOCK		SC2723_WDG(0x20)
#define WDG_UNLOCK_KEY		0xe551
#define WDG_TICKS_50MS		(32768 * 50 / 1000)

static void __iomem *sec_debug;

static int sprd_restart(struct notifier_block *nb, unsigned long mode,
			void *data)
{
	const char *cmd = data;
	u32 hwrst = HWRST_SPECIAL, sec = SEC_MODE_NONE, v;
	int i;

	if (cmd && !strcmp(cmd, "recovery")) {
		hwrst = HWRST_RECOVERY;
		sec = SEC_MODE_RECOVERY;
	} else if (cmd && !strcmp(cmd, "download")) {
		hwrst = HWRST_NORMAL;
		sec = SEC_MODE_DOWNLOAD;
	} else if (cmd) {
		hwrst = HWRST_NORMAL;
	}
	pr_emerg("sprd-restart: cmd %s, upload magic was %08x\n",
		 cmd ?: "(none)", sec_debug ? readl(sec_debug) : 0);

	if (sec_debug) {
		writel(0, sec_debug + SEC_UPLOAD_MAGIC);
		writel(SEC_NO_LPM, sec_debug + SEC_INFORM2);
		writel(SEC_MODE(sec), sec_debug + SEC_INFORM3);
	}
	sprd_adi_write(GLB_POR_RST_MONITOR, hwrst);

	sprd_adi_update(GLB_ARM_MODULE_EN, 0, GLB_WDG_EN);
	sprd_adi_update(GLB_RTC_CLK_EN, 0, GLB_WDG_EN);
	sprd_adi_write(WDG_LOCK, WDG_UNLOCK_KEY);
	sprd_adi_update(WDG_CTRL, 0, WDG_NEW_VER_EN);
	sprd_adi_write(WDG_LOAD_HIGH, WDG_TICKS_50MS >> 16);
	sprd_adi_write(WDG_LOAD_LOW, WDG_TICKS_50MS & 0xffff);
	for (i = 0; i < 10000; i++)
		if (sprd_adi_read(WDG_INT_RAW, &v) || !(v & WDG_LD_BUSY))
			break;
	sprd_adi_update(WDG_CTRL, 0, WDG_CNT_EN | WDG_RST_EN);
	sprd_adi_write(WDG_LOCK, ~WDG_UNLOCK_KEY & 0xffff);

	mdelay(1000);
	pr_emerg("sprd-restart: PMIC watchdog did not reset the system\n");
	return NOTIFY_DONE;
}

static struct notifier_block sprd_restart_nb = {
	.notifier_call = sprd_restart,
	.priority = 128,
};

static int __init sprd_restart_init(void)
{
	if (!of_machine_is_compatible("sprd,sc7730"))
		return 0;
	sec_debug = ioremap(SEC_DEBUG_PHYS, 0x20);
	return register_restart_handler(&sprd_restart_nb);
}
arch_initcall(sprd_restart_init);
