// SPDX-License-Identifier: GPL-2.0
/*
 * I2C controller of the Spreadtrum SC8830/SC7730 ("shark") SoCs.
 *
 * A byte-at-a-time engine: START, every data byte and STOP each go through
 * the CMD register, and the driver polls for completion, as the vendor 3.10
 * driver (drivers/i2c/busses/i2c-sprd.c) does. It has nothing in common
 * with the FIFO based controller that mainline i2c-sprd.c handles (SC9860
 * and later).
 *
 * Until there is a clock/reset driver for this SoC, the controller's AP APB
 * clock gate and soft reset (same bit in EB at +0 and RST at +4 of the
 * optional "ap-apb" region, bit number in "sprd,ap-apb-bit") are handled
 * here. The bootloader leaves the bus unused, so both must be set up.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define I2C_CTL			0x00
#define I2C_CTL_INT		BIT(0)
#define I2C_CTL_IE		BIT(3)
#define I2C_CTL_EN		BIT(4)
#define I2C_CMD			0x04
#define I2C_CMD_INT_ACK		BIT(0)
#define I2C_CMD_TX_ACK		BIT(1)	/* send NACK after this read */
#define I2C_CMD_WRITE		BIT(2)
#define I2C_CMD_READ		BIT(3)
#define I2C_CMD_STOP		BIT(4)
#define I2C_CMD_START		BIT(5)
#define I2C_CMD_ACK		BIT(6)	/* set while no ACK was received */
#define I2C_CMD_BUSY		BIT(7)
#define I2C_CMD_DATA(x)		((x) << 8)
#define I2C_CLKD0		0x08
#define I2C_CLKD1		0x0c

#define AP_APB_EB		0x00
#define AP_APB_RST		0x04

#define SC8830_I2C_TIMEOUT_US	5000

struct sc8830_i2c {
	struct i2c_adapter adap;
	void __iomem *base;
	unsigned long src_rate;
	u32 bus_rate;
};

static void sc8830_i2c_hw_init(struct sc8830_i2c *i2c)
{
	u32 div = i2c->src_rate / (4 * i2c->bus_rate) - 1;
	u32 ctl = readl(i2c->base + I2C_CTL);

	writel(ctl & ~(I2C_CTL_EN | I2C_CTL_IE), i2c->base + I2C_CTL);
	writel(div & 0xffff, i2c->base + I2C_CLKD0);
	writel(div >> 16, i2c->base + I2C_CLKD1);
	/* IE as in the vendor driver; we poll and never request the IRQ */
	writel(ctl | I2C_CTL_EN | I2C_CTL_IE, i2c->base + I2C_CTL);
	writel(I2C_CMD_INT_ACK, i2c->base + I2C_CMD);
}

static int sc8830_i2c_cmd(struct sc8830_i2c *i2c, u32 cmd)
{
	void __iomem *reg = i2c->base + I2C_CMD;
	u32 v;
	int ret;

	writel(cmd, reg);
	ret = readl_poll_timeout_atomic(i2c->base + I2C_CTL, v, v & I2C_CTL_INT,
					1, SC8830_I2C_TIMEOUT_US);
	if (ret)
		return ret;

	/* acknowledge the interrupt once idle, keeping a received byte */
	readl_poll_timeout_atomic(reg, v, !(v & I2C_CMD_BUSY), 1,
				  SC8830_I2C_TIMEOUT_US);
	writel((readl(reg) & 0xff00) | I2C_CMD_INT_ACK, reg);

	ret = readl_poll_timeout_atomic(reg, v, !(v & I2C_CMD_ACK), 1,
					SC8830_I2C_TIMEOUT_US);
	return ret ? -ENXIO : 0;
}

static int sc8830_i2c_msg(struct sc8830_i2c *i2c, struct i2c_msg *msg,
			  bool last)
{
	bool rd = msg->flags & I2C_M_RD;
	int i, ret;

	ret = sc8830_i2c_cmd(i2c, I2C_CMD_DATA(i2c_8bit_addr_from_msg(msg)) |
			     I2C_CMD_START | I2C_CMD_WRITE);
	if (ret)
		return ret;

	for (i = 0; i < msg->len; i++) {
		bool final = i == msg->len - 1;

		if (rd) {
			/* like the vendor: every read message ends with STOP */
			ret = sc8830_i2c_cmd(i2c, I2C_CMD_READ | (final ?
					     I2C_CMD_STOP | I2C_CMD_TX_ACK : 0));
			if (ret)
				return ret;
			msg->buf[i] = readl(i2c->base + I2C_CMD) >> 8;
		} else {
			ret = sc8830_i2c_cmd(i2c, I2C_CMD_DATA(msg->buf[i]) |
					     I2C_CMD_WRITE |
					     (final && last ? I2C_CMD_STOP : 0));
			if (ret)
				return ret;
		}
	}
	return 0;
}

static int sc8830_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   int num)
{
	struct sc8830_i2c *i2c = i2c_get_adapdata(adap);
	int i, ret = 0;

	for (i = 0; i < num && !ret; i++)
		ret = sc8830_i2c_msg(i2c, &msgs[i], i == num - 1);

	if (ret) {
		/* leave the engine in a known state for the next transfer */
		sc8830_i2c_hw_init(i2c);
		return ret;
	}
	return num;
}

static u32 sc8830_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

/* polling only, so the same path works in atomic context */
static const struct i2c_algorithm sc8830_i2c_algo = {
	.xfer		= sc8830_i2c_xfer,
	.xfer_atomic	= sc8830_i2c_xfer,
	.functionality	= sc8830_i2c_func,
};

/* A write without data would need a bare STOP, which the engine lacks */
static const struct i2c_adapter_quirks sc8830_i2c_quirks = {
	.flags = I2C_AQ_NO_ZERO_LEN,
};

static void sc8830_i2c_ap_apb_init(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *apb;
	u32 bit;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ap-apb");
	if (!res || of_property_read_u32(pdev->dev.of_node, "sprd,ap-apb-bit",
					 &bit))
		return;
	/* shared with other blocks, so no request_mem_region */
	apb = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!apb)
		return;
	writel(readl(apb + AP_APB_EB) | BIT(bit), apb + AP_APB_EB);
	writel(readl(apb + AP_APB_RST) | BIT(bit), apb + AP_APB_RST);
	udelay(10);
	writel(readl(apb + AP_APB_RST) & ~BIT(bit), apb + AP_APB_RST);
}

static int sc8830_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc8830_i2c *i2c;
	struct clk *clk;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->base = devm_platform_ioremap_resource_byname(pdev, "i2c");
	if (IS_ERR(i2c->base))
		return PTR_ERR(i2c->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "no source clock\n");
	i2c->src_rate = clk_get_rate(clk);

	i2c->bus_rate = I2C_MAX_STANDARD_MODE_FREQ;
	of_property_read_u32(dev->of_node, "clock-frequency", &i2c->bus_rate);
	if (!i2c->src_rate || !i2c->bus_rate ||
	    i2c->src_rate < 4 * i2c->bus_rate)
		return dev_err_probe(dev, -EINVAL, "bad clock rates\n");

	sc8830_i2c_ap_apb_init(pdev);
	sc8830_i2c_hw_init(i2c);

	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &sc8830_i2c_algo;
	i2c->adap.quirks = &sc8830_i2c_quirks;
	i2c->adap.dev.parent = dev;
	device_set_node(&i2c->adap.dev, dev_fwnode(dev));
	strscpy(i2c->adap.name, dev_name(dev), sizeof(i2c->adap.name));
	i2c_set_adapdata(&i2c->adap, i2c);

	dev_info(dev, "%u Hz bus from %lu Hz\n", i2c->bus_rate, i2c->src_rate);
	return devm_i2c_add_adapter(dev, &i2c->adap);
}

static const struct of_device_id sc8830_i2c_of_match[] = {
	{ .compatible = "sprd,sc8830-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc8830_i2c_of_match);

static struct platform_driver sc8830_i2c_driver = {
	.probe = sc8830_i2c_probe,
	.driver = {
		.name = "i2c-sprd-sc8830",
		.of_match_table = sc8830_i2c_of_match,
	},
};
module_platform_driver(sc8830_i2c_driver);

MODULE_DESCRIPTION("Spreadtrum SC8830/SC7730 I2C controller");
MODULE_LICENSE("GPL");
