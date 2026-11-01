// SPDX-License-Identifier: GPL-2.0
/*
 * Power up the SC7730 USB PHY so the generic dwc2 driver can run the
 * controller in device mode. Bring-up hack until there are proper clock,
 * PHY and SC2723 regulator drivers.
 *
 * Sequence and register map from the vendor 3.10 kernel
 * (drivers/usb/gadget/dwc_otg/usb_hw.c: udc_enable() + usb_phy_init(),
 * headers mach/chip_x30g/ with the tshark2 AON block):
 *
 *   1. SC2723 LDO_USB on: ANA_GLB LDO_PD_CTRL bit 8, behind the ADI bus
 *   2. USB PHY out of power-down: AON_APB PWR_CTRL bit 0
 *   3. USB AHB clock on: AP_AHB EB bit 4
 *   4. pulse USB, UTMI and PHY soft resets: AP_AHB RST bits 5..7
 *   5. AP_APB USB_PHY_CTRL bit 9 and the board's PHY tune value
 *
 * The vendor driver forces the B-session valid in GOTGCTL; with dwc2 the
 * "usb-role-switch" + "role-switch-default-mode" properties do the same.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/printk.h>

#define ADI_PHYS		0x40030000
#define ADI_RD_CMD		0x24
#define ADI_RD_DATA		0x28
#define ADI_RD_BUSY		BIT(31)
#define ADI_FIFO_STS		0x2c
#define ADI_FIFO_EMPTY		BIT(10)

/* SC2723 global registers: ADI slave window (0x40038000) + 0x800 */
#define ANA_GLB_PHYS		0x40038800
#define ANA_LDO_PD_CTRL		0x14
#define ANA_LDO_USB_PD		BIT(8)
#define ANA_LDO_V_CTRL9		0x3c	/* bits 7:0 LDO_USB_V */

#define AON_APB_PHYS		0x402e0000
#define AON_APB_EB0		0x00
#define AON_APB_PWR_CTRL	0x24
#define AON_USB_PHY_PD		BIT(0)

#define AP_AHB_PHYS		0x20d00000
#define AP_AHB_EB		0x00
#define AP_AHB_USB_EB		BIT(4)
#define AP_AHB_RST		0x04
#define AP_AHB_USB_RSTS		(BIT(5) | BIT(6) | BIT(7))

#define AP_APB_PHYS		0x71300000
#define AP_APB_USB_PHY_TUNE	0x3000
#define AP_APB_USB_PHY_CTRL	0x3008
#define AP_APB_PHY_CTRL_BIT9	BIT(9)

#define USB_PHYS		0x20200000
#define DWC2_GSNPSID		0x40

/* "tune_value" of the vendor usb@f5116000 node for gtelwifi */
#define GTELWIFI_PHY_TUNE	0x44076e33

static void __iomem *adi, *ana;

static int __init adi_read(u32 off, u32 *val)
{
	int i;

	writel(ANA_GLB_PHYS + off, adi + ADI_RD_CMD);
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

static int __init adi_write(u32 off, u32 val)
{
	int i;

	writel(val, ana + off);
	for (i = 0; i < 1000; i++) {
		if (readl(adi + ADI_FIFO_STS) & ADI_FIFO_EMPTY)
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

static void __init rmw(void __iomem *reg, u32 clr, u32 set)
{
	writel((readl(reg) & ~clr) | set, reg);
}

static int __init sprd_usb_bringup(void)
{
	void __iomem *aon, *ahb, *apb, *usb;
	struct device_node *np;
	u32 pd = 0, pd_new = 0, vusb = 0;
	int ret;

	if (!of_machine_is_compatible("sprd,sc7730"))
		return 0;
	np = of_find_compatible_node(NULL, NULL, "snps,dwc2");
	if (!np)
		return 0;
	ret = of_device_is_available(np);
	of_node_put(np);
	if (!ret)
		return 0;

	adi = ioremap(ADI_PHYS, 0x100);
	ana = ioremap(ANA_GLB_PHYS, 0x100);
	aon = ioremap(AON_APB_PHYS, 0x100);
	ahb = ioremap(AP_AHB_PHYS, 0x100);
	apb = ioremap(AP_APB_PHYS + AP_APB_USB_PHY_TUNE, 0x10);
	usb = ioremap(USB_PHYS, 0x100);
	if (!adi || !ana || !aon || !ahb || !apb || !usb) {
		pr_err("sprd-usb: ioremap failed\n");
		goto out;
	}

	pr_info("sprd-usb: before: AON_EB0 %08x PWR_CTRL %08x AHB_EB %08x PHY_CTRL %08x TUNE %08x\n",
		readl(aon + AON_APB_EB0), readl(aon + AON_APB_PWR_CTRL),
		readl(ahb + AP_AHB_EB), readl(apb + 8), readl(apb));

	/* 1. LDO_USB */
	ret = adi_read(ANA_LDO_PD_CTRL, &pd);
	adi_read(ANA_LDO_V_CTRL9, &vusb);
	if (!ret && (pd & ANA_LDO_USB_PD))
		ret = adi_write(ANA_LDO_PD_CTRL, pd & ~ANA_LDO_USB_PD);
	if (!ret)
		ret = adi_read(ANA_LDO_PD_CTRL, &pd_new);
	pr_info("sprd-usb: LDO_PD_CTRL %04x -> %04x (%d), LDO_V_CTRL9 %04x\n",
		pd, pd_new, ret, vusb);
	mdelay(10);

	/* 2.-4. PHY power, AHB clock, resets */
	rmw(aon + AON_APB_PWR_CTRL, AON_USB_PHY_PD, 0);
	rmw(ahb + AP_AHB_EB, 0, AP_AHB_USB_EB);
	mdelay(2);
	rmw(ahb + AP_AHB_RST, 0, AP_AHB_USB_RSTS);
	mdelay(5);
	rmw(ahb + AP_AHB_RST, AP_AHB_USB_RSTS, 0);
	mdelay(3);

	/* 5. PHY control and tune */
	rmw(apb + 8, 0, AP_APB_PHY_CTRL_BIT9);
	writel(GTELWIFI_PHY_TUNE, apb);

	pr_info("sprd-usb: after: PWR_CTRL %08x AHB_EB %08x PHY_CTRL %08x TUNE %08x GSNPSID %08x\n",
		readl(aon + AON_APB_PWR_CTRL), readl(ahb + AP_AHB_EB),
		readl(apb + 8), readl(apb), readl(usb + DWC2_GSNPSID));
out:
	if (usb)
		iounmap(usb);
	if (apb)
		iounmap(apb);
	if (ahb)
		iounmap(ahb);
	if (aon)
		iounmap(aon);
	if (ana)
		iounmap(ana);
	if (adi)
		iounmap(adi);
	return 0;
}
subsys_initcall(sprd_usb_bringup);
