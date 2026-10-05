// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon SD5116 SPI flash controller (SFC300) driver
 *
 * The controller exposes a command interface: opcode, address and up to
 * 64 bytes of data are staged in registers and a single command register
 * write starts the transfer. Only single-bit transfers with 3-byte
 * addresses are supported.
 *
 * Register usage was determined by observing the vendor bootloader.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

#define SFC_INT_RAW_STATUS	0x120
#define SFC_INT_CLEAR		0x12c
#define SFC_INT_OP_DONE		BIT(0)

#define SFC_CMD_CONFIG		0x300
#define SFC_CMD_START		BIT(0)
#define SFC_CMD_SEL_CS		BIT(1)
#define SFC_CMD_ADDR_EN		BIT(3)
#define SFC_CMD_DUMMY_CNT	GENMASK(6, 4)	/* in bytes */
#define SFC_CMD_DATA_EN		BIT(7)
#define SFC_CMD_RW_READ		BIT(8)
#define SFC_CMD_DATA_CNT	GENMASK(14, 9)

#define SFC_CMD_INS		0x308
#define SFC_CMD_ADDR		0x30c
#define SFC_CMD_ADDR_MASK	GENMASK(29, 0)
#define SFC_CMD_DATABUF		0x400
#define SFC_DATABUF_SIZE	64

#define SFC_NUM_CS		2
#define SFC_TIMEOUT_US		100000

struct sd5116_sfc {
	void __iomem *base;
};

static bool sd5116_sfc_supports_op(struct spi_mem *mem,
				   const struct spi_mem_op *op)
{
	if (op->cmd.buswidth != 1 || op->cmd.nbytes != 1 || op->cmd.dtr)
		return false;
	if (op->addr.nbytes &&
	    (op->addr.nbytes != 3 || op->addr.buswidth != 1))
		return false;
	if (op->dummy.nbytes &&
	    (op->dummy.buswidth != 1 ||
	     op->dummy.nbytes > FIELD_MAX(SFC_CMD_DUMMY_CNT)))
		return false;
	if (op->data.nbytes && op->data.buswidth != 1)
		return false;

	return spi_mem_default_supports_op(mem, op);
}

static int sd5116_sfc_adjust_op_size(struct spi_mem *mem,
				     struct spi_mem_op *op)
{
	op->data.nbytes = min(op->data.nbytes, SFC_DATABUF_SIZE);
	return 0;
}

static void sd5116_sfc_write_buf(struct sd5116_sfc *sfc, const u8 *buf,
				 unsigned int len)
{
	unsigned int i;
	u32 val;

	for (i = 0; i < len; i += 4) {
		val = 0;
		memcpy(&val, buf + i, min(4U, len - i));
		writel(val, sfc->base + SFC_CMD_DATABUF + i);
	}
}

static void sd5116_sfc_read_buf(struct sd5116_sfc *sfc, u8 *buf,
				unsigned int len)
{
	unsigned int i;
	u32 val;

	for (i = 0; i < len; i += 4) {
		val = readl(sfc->base + SFC_CMD_DATABUF + i);
		memcpy(buf + i, &val, min(4U, len - i));
	}
}

static int sd5116_sfc_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct sd5116_sfc *sfc = spi_controller_get_devdata(mem->spi->controller);
	unsigned int len = op->data.nbytes;
	u32 cfg = SFC_CMD_START;
	u32 status;
	int ret;

	if (len > SFC_DATABUF_SIZE)
		return -EINVAL;

	if (spi_get_chipselect(mem->spi, 0))
		cfg |= SFC_CMD_SEL_CS;

	writel(op->cmd.opcode, sfc->base + SFC_CMD_INS);

	if (op->addr.nbytes) {
		writel(op->addr.val & SFC_CMD_ADDR_MASK,
		       sfc->base + SFC_CMD_ADDR);
		cfg |= SFC_CMD_ADDR_EN;
	}

	cfg |= FIELD_PREP(SFC_CMD_DUMMY_CNT, op->dummy.nbytes);

	if (len) {
		cfg |= SFC_CMD_DATA_EN | FIELD_PREP(SFC_CMD_DATA_CNT, len - 1);
		if (op->data.dir == SPI_MEM_DATA_IN)
			cfg |= SFC_CMD_RW_READ;
		else
			sd5116_sfc_write_buf(sfc, op->data.buf.out, len);
	}

	writel(SFC_INT_OP_DONE, sfc->base + SFC_INT_CLEAR);
	writel(cfg, sfc->base + SFC_CMD_CONFIG);

	ret = readl_poll_timeout(sfc->base + SFC_INT_RAW_STATUS, status,
				 status & SFC_INT_OP_DONE, 0, SFC_TIMEOUT_US);
	writel(SFC_INT_OP_DONE, sfc->base + SFC_INT_CLEAR);
	if (ret)
		return ret;

	if (len && op->data.dir == SPI_MEM_DATA_IN)
		sd5116_sfc_read_buf(sfc, op->data.buf.in, len);

	return 0;
}

static const struct spi_controller_mem_ops sd5116_sfc_mem_ops = {
	.adjust_op_size = sd5116_sfc_adjust_op_size,
	.supports_op = sd5116_sfc_supports_op,
	.exec_op = sd5116_sfc_exec_op,
};

static int sd5116_sfc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct sd5116_sfc *sfc;
	struct clk *clk;

	ctlr = devm_spi_alloc_host(dev, sizeof(*sfc));
	if (!ctlr)
		return -ENOMEM;

	sfc = spi_controller_get_devdata(ctlr);

	sfc->base = devm_platform_ioremap_resource_byname(pdev, "control");
	if (IS_ERR(sfc->base))
		return PTR_ERR(sfc->base);

	clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get clock\n");

	ctlr->num_chipselect = SFC_NUM_CS;
	ctlr->mem_ops = &sd5116_sfc_mem_ops;
	ctlr->dev.of_node = dev->of_node;

	return devm_spi_register_controller(dev, ctlr);
}

static const struct of_device_id sd5116_sfc_of_match[] = {
	{ .compatible = "hisilicon,sd5116-sfc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sd5116_sfc_of_match);

static struct platform_driver sd5116_sfc_driver = {
	.probe = sd5116_sfc_probe,
	.driver = {
		.name = "sd5116-sfc",
		.of_match_table = sd5116_sfc_of_match,
	},
};
module_platform_driver(sd5116_sfc_driver);

MODULE_AUTHOR("Mario Moura <mm@mariomoura.com>");
MODULE_DESCRIPTION("HiSilicon SD5116 SPI flash controller driver");
MODULE_LICENSE("GPL");
