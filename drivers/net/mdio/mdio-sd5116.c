// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon SD5116 MDIO controller driver
 *
 * Register usage was determined by reverse engineering the vendor firmware.
 */

#include <linux/bitfield.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/platform_device.h>

#define MDIO_MODE		0x00
#define MDIO_MODE_MASK		GENMASK(2, 0)
#define MDIO_CMD		0x04
#define MDIO_CMD_START		BIT(0)
#define MDIO_CMD_WRITE		BIT(1)
#define MDIO_CMD_PHY		GENMASK(6, 2)
#define MDIO_CMD_REG		GENMASK(11, 7)
#define MDIO_CMD_DATA		GENMASK(27, 12)
#define MDIO_RDATA		0x0c
#define MDIO_RDATA_MASK		GENMASK(15, 0)
#define MDIO_STATUS		0x10
#define MDIO_STATUS_BUSY	GENMASK(2, 0)
#define MDIO_STATUS_DONE	GENMASK(4, 3)

#define MDIO_TIMEOUT_US		10000

struct sd5116_mdio {
	void __iomem *base;
};

static int sd5116_mdio_wait(struct sd5116_mdio *priv, u32 done)
{
	u32 val;

	return readl_poll_timeout(priv->base + MDIO_STATUS, val,
				  !(val & MDIO_STATUS_BUSY) &&
				  (!done || (val & done)), 10, MDIO_TIMEOUT_US);
}

static int sd5116_mdio_xfer(struct sd5116_mdio *priv, u32 cmd)
{
	int ret;

	ret = sd5116_mdio_wait(priv, 0);
	if (ret)
		return ret;

	writel(MDIO_CMD_START | cmd, priv->base + MDIO_CMD);

	return sd5116_mdio_wait(priv, MDIO_STATUS_DONE);
}

static int sd5116_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct sd5116_mdio *priv = bus->priv;
	int ret;

	ret = sd5116_mdio_xfer(priv, FIELD_PREP(MDIO_CMD_PHY, addr) |
				     FIELD_PREP(MDIO_CMD_REG, regnum));
	if (ret)
		return ret;

	return readl(priv->base + MDIO_RDATA) & MDIO_RDATA_MASK;
}

static int sd5116_mdio_write(struct mii_bus *bus, int addr, int regnum,
			     u16 val)
{
	struct sd5116_mdio *priv = bus->priv;

	return sd5116_mdio_xfer(priv, MDIO_CMD_WRITE |
				      FIELD_PREP(MDIO_CMD_PHY, addr) |
				      FIELD_PREP(MDIO_CMD_REG, regnum) |
				      FIELD_PREP(MDIO_CMD_DATA, val));
}

static int sd5116_mdio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sd5116_mdio *priv;
	struct mii_bus *bus;

	bus = devm_mdiobus_alloc_size(dev, sizeof(*priv));
	if (!bus)
		return -ENOMEM;

	priv = bus->priv;
	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	bus->name = "sd5116-mdio";
	bus->read = sd5116_mdio_read;
	bus->write = sd5116_mdio_write;
	bus->parent = dev;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));

	writel(readl(priv->base + MDIO_MODE) & ~MDIO_MODE_MASK,
	       priv->base + MDIO_MODE);

	return devm_of_mdiobus_register(dev, bus, dev->of_node);
}

static const struct of_device_id sd5116_mdio_of_match[] = {
	{ .compatible = "hisilicon,sd5116-mdio" },
	{ }
};
MODULE_DEVICE_TABLE(of, sd5116_mdio_of_match);

static struct platform_driver sd5116_mdio_driver = {
	.probe = sd5116_mdio_probe,
	.driver = {
		.name = "sd5116-mdio",
		.of_match_table = sd5116_mdio_of_match,
	},
};
module_platform_driver(sd5116_mdio_driver);

MODULE_AUTHOR("Mario Moura <mm@mariomoura.com>");
MODULE_DESCRIPTION("HiSilicon SD5116 MDIO controller driver");
MODULE_LICENSE("GPL");
