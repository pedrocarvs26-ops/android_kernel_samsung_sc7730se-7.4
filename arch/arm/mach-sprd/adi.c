// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal access to the SC2723 PMIC behind the Spreadtrum ADI bus, for
 * the bring-up code in this directory (USB LDO, restart).
 *
 * Protocol from the vendor 3.10 kernel (arch/arm/mach-sc/adi.c), register
 * offsets as in mainline drivers/spi/spi-sprd-adi.c:
 *   read:  write the slave's physical address to RD_CMD, poll RD_DATA
 *          until bit 31 clears, the value is in bits 15:0
 *   write: plain MMIO store into the slave window, then wait until the
 *          write FIFO is empty
 */

#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/spinlock.h>

#include "adi.h"

#define ADI_PHYS		0x40030000
#define ADI_RD_CMD		0x24
#define ADI_RD_DATA		0x28
#define ADI_RD_BUSY		BIT(31)
#define ADI_FIFO_STS		0x2c
#define ADI_FIFO_EMPTY		BIT(10)
#define ADI_SLAVE_PHYS		0x40038000
#define ADI_SLAVE_SIZE		0x1000

static void __iomem *adi, *slave;
static DEFINE_RAW_SPINLOCK(adi_lock);

static int __adi_read(u32 reg, u32 *val)
{
	int i;

	writel(ADI_SLAVE_PHYS + reg, adi + ADI_RD_CMD);
	for (i = 0; i < 2000; i++) {
		u32 v = readl(adi + ADI_RD_DATA);

		if (!(v & ADI_RD_BUSY)) {
			*val = v & 0xffff;
			return 0;
		}
		udelay(1);
	}
	return -ETIMEDOUT;
}

static int __adi_write(u32 reg, u32 val)
{
	int i;

	writel(val, slave + reg);
	for (i = 0; i < 1000; i++) {
		if (readl(adi + ADI_FIFO_STS) & ADI_FIFO_EMPTY)
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

int sprd_adi_read(u32 reg, u32 *val)
{
	unsigned long flags;
	int ret;

	if (!adi)
		return -ENODEV;
	raw_spin_lock_irqsave(&adi_lock, flags);
	ret = __adi_read(reg, val);
	raw_spin_unlock_irqrestore(&adi_lock, flags);
	return ret;
}

int sprd_adi_write(u32 reg, u32 val)
{
	unsigned long flags;
	int ret;

	if (!adi)
		return -ENODEV;
	raw_spin_lock_irqsave(&adi_lock, flags);
	ret = __adi_write(reg, val);
	raw_spin_unlock_irqrestore(&adi_lock, flags);
	return ret;
}

int sprd_adi_update(u32 reg, u32 clr, u32 set)
{
	unsigned long flags;
	u32 v;
	int ret;

	if (!adi)
		return -ENODEV;
	raw_spin_lock_irqsave(&adi_lock, flags);
	ret = __adi_read(reg, &v);
	if (!ret)
		ret = __adi_write(reg, (v & ~clr) | set);
	raw_spin_unlock_irqrestore(&adi_lock, flags);
	return ret;
}

static int __init sprd_adi_init(void)
{
	if (!of_machine_is_compatible("sprd,sc7730"))
		return 0;
	adi = ioremap(ADI_PHYS, 0x100);
	slave = ioremap(ADI_SLAVE_PHYS, ADI_SLAVE_SIZE);
	if (!adi || !slave) {
		pr_err("sprd-adi: ioremap failed\n");
		if (adi)
			iounmap(adi);
		adi = NULL;
		return -ENOMEM;
	}
	return 0;
}
early_initcall(sprd_adi_init);

#ifdef CONFIG_DEBUG_FS
/*
 * debugfs sprd-adi/{address,value}: PMIC registers from user space for
 * bring-up (audio, camera rails), under the same lock as the drivers.
 * /dev/mem must not be used for the ADI: a read is a command sequence and
 * would race the fuel gauge polling.
 */
static u32 adi_dbg_addr;

static int adi_dbg_get(void *data, u64 *val)
{
	u32 v;
	int ret;

	if (adi_dbg_addr >= ADI_SLAVE_SIZE || adi_dbg_addr & 3)
		return -EINVAL;
	ret = sprd_adi_read(adi_dbg_addr, &v);
	*val = v;
	return ret;
}

static int adi_dbg_set(void *data, u64 val)
{
	if (adi_dbg_addr >= ADI_SLAVE_SIZE || adi_dbg_addr & 3)
		return -EINVAL;
	return sprd_adi_write(adi_dbg_addr, val);
}
DEFINE_DEBUGFS_ATTRIBUTE(adi_dbg_fops, adi_dbg_get, adi_dbg_set, "0x%08llx\n");

static int __init sprd_adi_debugfs_init(void)
{
	struct dentry *dir;

	if (!adi)
		return 0;
	dir = debugfs_create_dir("sprd-adi", NULL);
	debugfs_create_x32("address", 0600, dir, &adi_dbg_addr);
	debugfs_create_file_unsafe("value", 0600, dir, NULL, &adi_dbg_fops);
	return 0;
}
late_initcall(sprd_adi_debugfs_init);
#endif
