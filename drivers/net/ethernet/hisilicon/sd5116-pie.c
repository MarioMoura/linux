// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon SD5116 PIE (Packet Interface Engine) Ethernet driver
 *
 * The PIE is the DMA engine between the CPU and port 4 of the on-chip
 * switch. It is used as the DSA conduit of that switch, so frames are passed
 * through unmodified. The engine reads and writes only aligned buffers, so
 * frames are copied through fixed coherent bounce buffers.
 */

#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>

#define PIE_CTRL		0x000
#define PIE_CTRL_EN		BIT(0)
#define PIE_IRQ_STATUS		0x004	/* write 1 to clear */
#define PIE_IRQ_MASK		0x008	/* 1 = masked */
#define PIE_IRQ_RX		BIT(0)
#define PIE_IRQ_TX		BIT(8)
#define PIE_IRQ_ALL		GENMASK(19, 0)
#define PIE_CFG			0x010
#define PIE_STATUS		0x014
#define PIE_STATUS_READY	BIT(0)
#define PIE_CHAN_EN(n)		(0x01c + (n) * 4)
#define PIE_NUM_CHAN		11
#define PIE_RX_RING_BASE	0x100
#define PIE_RX_RING_ORDER	0x104
#define PIE_RX_PROD		0x108
#define PIE_RX_CONS		0x10c
#define PIE_MODE		0x180
#define PIE_MODE_CHAN_MASK	GENMASK(7, 0)
#define PIE_TX_RING_BASE	0x240
#define PIE_TX_RING_ORDER	0x244
#define PIE_TX_PROD		0x248
#define PIE_TX_CONS		0x24c

#define RING_ORDER		3
#define RING_SIZE		BIT(RING_ORDER)
#define RING_MASK		(RING_SIZE - 1)
#define TX_BUF_SIZE		2048
/* the engine has no RX size limit; fit the longest length it can report */
#define RX_BUF_SIZE		SZ_16K

struct pie_desc {
	__le32 ctrl;
	__le32 addr;
	__le32 info;
	__le32 meta;
};

#define DESC_OWN		BIT(15)
#define DESC_TX_FWD		BIT(14)	/* without it frames never enter the switch */
#define DESC_LEN		GENMASK(13, 0)

struct sd5116_pie {
	void __iomem *base;
	struct device *dev;
	struct net_device *ndev;
	struct napi_struct napi;

	struct pie_desc *tx_ring;
	dma_addr_t tx_ring_dma;
	void *tx_buf;
	dma_addr_t tx_buf_dma;
	unsigned int tx_len[RING_SIZE];
	unsigned int tx_prod;
	unsigned int tx_cons;

	struct pie_desc *rx_ring;
	dma_addr_t rx_ring_dma;
	void *rx_buf;
	dma_addr_t rx_buf_dma;
	unsigned int rx_prod;
	unsigned int rx_cons;
};

static void sd5116_pie_arm_rx(struct sd5116_pie *pie, unsigned int idx)
{
	struct pie_desc *desc = &pie->rx_ring[idx];

	desc->addr = cpu_to_le32(pie->rx_buf_dma + idx * RX_BUF_SIZE);
	desc->info = 0;
	desc->meta = 0;
	/* descriptor body must be visible before OWN is set */
	wmb();
	desc->ctrl = cpu_to_le32(DESC_OWN);
}

static int sd5116_pie_alloc_rings(struct sd5116_pie *pie)
{
	size_t ring = RING_SIZE * sizeof(struct pie_desc);

	/* the engine cannot be stopped, so allocate for the device lifetime */
	pie->tx_ring = dmam_alloc_coherent(pie->dev, ring, &pie->tx_ring_dma,
					   GFP_KERNEL);
	pie->rx_ring = dmam_alloc_coherent(pie->dev, ring, &pie->rx_ring_dma,
					   GFP_KERNEL);
	pie->tx_buf = dmam_alloc_coherent(pie->dev, RING_SIZE * TX_BUF_SIZE,
					  &pie->tx_buf_dma, GFP_KERNEL);
	pie->rx_buf = dmam_alloc_coherent(pie->dev, RING_SIZE * RX_BUF_SIZE,
					  &pie->rx_buf_dma, GFP_KERNEL);
	if (!pie->tx_ring || !pie->rx_ring || !pie->tx_buf || !pie->rx_buf)
		return -ENOMEM;

	return 0;
}

static int sd5116_pie_hw_start(struct sd5116_pie *pie)
{
	u32 val;
	int i, ret;

	ret = readl_poll_timeout(pie->base + PIE_STATUS, val,
				 val & PIE_STATUS_READY, 1000, 100000);
	if (ret)
		return ret;

	writel(readl(pie->base + PIE_CTRL) & ~PIE_CTRL_EN, pie->base + PIE_CTRL);
	writel(PIE_IRQ_ALL, pie->base + PIE_IRQ_MASK);
	writel(PIE_IRQ_ALL, pie->base + PIE_IRQ_STATUS);

	for (i = 0; i < PIE_NUM_CHAN; i++)
		writel(1, pie->base + PIE_CHAN_EN(i));
	writel(readl(pie->base + PIE_MODE) & ~PIE_MODE_CHAN_MASK,
	       pie->base + PIE_MODE);
	writel(0x100, pie->base + PIE_CFG);

	memset(pie->tx_ring, 0, RING_SIZE * sizeof(struct pie_desc));
	for (i = 0; i < RING_SIZE; i++)
		sd5116_pie_arm_rx(pie, i);

	writel(pie->rx_ring_dma, pie->base + PIE_RX_RING_BASE);
	writel(RING_ORDER, pie->base + PIE_RX_RING_ORDER);
	writel(pie->tx_ring_dma, pie->base + PIE_TX_RING_BASE);
	writel(RING_ORDER, pie->base + PIE_TX_RING_ORDER);

	/* a new ring base does not reset the engine's consumer indices */
	pie->rx_cons = readl(pie->base + PIE_RX_CONS) & RING_MASK;
	pie->rx_prod = pie->rx_cons + RING_SIZE - 1;
	pie->tx_cons = readl(pie->base + PIE_TX_CONS) & RING_MASK;
	pie->tx_prod = pie->tx_cons;
	writel(pie->rx_prod & RING_MASK, pie->base + PIE_RX_PROD);
	writel(pie->tx_prod & RING_MASK, pie->base + PIE_TX_PROD);

	return 0;
}

static netdev_tx_t sd5116_pie_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct sd5116_pie *pie = netdev_priv(ndev);
	struct pie_desc *desc;
	unsigned int idx, len;

	if (pie->tx_prod - pie->tx_cons >= RING_SIZE - 1) {
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}

	if (skb_put_padto(skb, ETH_ZLEN)) {
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	len = skb->len;
	if (len > TX_BUF_SIZE) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	idx = pie->tx_prod & RING_MASK;
	skb_copy_bits(skb, 0, pie->tx_buf + idx * TX_BUF_SIZE, len);
	pie->tx_len[idx] = len;

	desc = &pie->tx_ring[idx];
	desc->addr = cpu_to_le32(pie->tx_buf_dma + idx * TX_BUF_SIZE);
	desc->info = 0;
	desc->meta = 0;
	/* descriptor body must be visible before OWN is set */
	wmb();
	desc->ctrl = cpu_to_le32(DESC_OWN | DESC_TX_FWD | len);

	pie->tx_prod++;
	writel(pie->tx_prod & RING_MASK, pie->base + PIE_TX_PROD);

	dev_consume_skb_any(skb);
	return NETDEV_TX_OK;
}

static void sd5116_pie_tx_complete(struct sd5116_pie *pie)
{
	struct net_device *ndev = pie->ndev;
	unsigned int idx;

	while (pie->tx_cons != pie->tx_prod) {
		idx = pie->tx_cons & RING_MASK;
		if (le32_to_cpu(pie->tx_ring[idx].ctrl) & DESC_OWN)
			break;

		ndev->stats.tx_bytes += pie->tx_len[idx];
		ndev->stats.tx_packets++;
		pie->tx_cons++;
	}

	if (netif_queue_stopped(ndev) &&
	    pie->tx_prod - pie->tx_cons < RING_SIZE - 1)
		netif_wake_queue(ndev);
}

static int sd5116_pie_poll(struct napi_struct *napi, int budget)
{
	struct sd5116_pie *pie = container_of(napi, struct sd5116_pie, napi);
	struct net_device *ndev = pie->ndev;
	int done = 0;

	sd5116_pie_tx_complete(pie);

	while (done < budget && pie->rx_cons != pie->rx_prod) {
		unsigned int idx = pie->rx_cons & RING_MASK;
		u32 ctrl = le32_to_cpu(pie->rx_ring[idx].ctrl);
		struct sk_buff *skb;
		unsigned int len;

		if (ctrl & DESC_OWN)
			break;
		/* read the buffer only after seeing OWN cleared */
		rmb();

		len = FIELD_GET(DESC_LEN, ctrl);
		if (len < ETH_ZLEN) {
			ndev->stats.rx_errors++;
			goto refill;
		}

		skb = napi_alloc_skb(napi, len);
		if (!skb) {
			ndev->stats.rx_dropped++;
			goto refill;
		}
		skb_put_data(skb, pie->rx_buf + idx * RX_BUF_SIZE, len);
		skb->protocol = eth_type_trans(skb, ndev);
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += len;
		napi_gro_receive(napi, skb);
refill:
		sd5116_pie_arm_rx(pie, idx);
		pie->rx_cons++;
		done++;
	}

	if (done) {
		pie->rx_prod += done;
		writel(pie->rx_prod & RING_MASK, pie->base + PIE_RX_PROD);
	}

	if (done < budget && napi_complete_done(napi, done))
		writel(PIE_IRQ_ALL & ~(PIE_IRQ_RX | PIE_IRQ_TX),
		       pie->base + PIE_IRQ_MASK);

	return done;
}

static irqreturn_t sd5116_pie_irq(int irq, void *dev_id)
{
	struct sd5116_pie *pie = dev_id;
	u32 status;

	status = readl(pie->base + PIE_IRQ_STATUS) & (PIE_IRQ_RX | PIE_IRQ_TX);
	if (!status)
		return IRQ_NONE;

	writel(PIE_IRQ_ALL, pie->base + PIE_IRQ_MASK);
	writel(status, pie->base + PIE_IRQ_STATUS);
	napi_schedule(&pie->napi);

	return IRQ_HANDLED;
}

static int sd5116_pie_open(struct net_device *ndev)
{
	struct sd5116_pie *pie = netdev_priv(ndev);
	int ret;

	ret = sd5116_pie_hw_start(pie);
	if (ret)
		return ret;

	napi_enable(&pie->napi);
	writel(PIE_IRQ_ALL & ~(PIE_IRQ_RX | PIE_IRQ_TX),
	       pie->base + PIE_IRQ_MASK);
	netif_carrier_on(ndev);
	netif_start_queue(ndev);

	return 0;
}

static int sd5116_pie_stop(struct net_device *ndev)
{
	struct sd5116_pie *pie = netdev_priv(ndev);
	int i;

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	writel(PIE_IRQ_ALL, pie->base + PIE_IRQ_MASK);
	writel(PIE_IRQ_ALL, pie->base + PIE_IRQ_STATUS);
	napi_disable(&pie->napi);

	/* take the RX descriptors back; the engine keeps running */
	for (i = 0; i < RING_SIZE; i++)
		pie->rx_ring[i].ctrl = 0;
	/* the engine must see the descriptors released before we return */
	wmb();

	return 0;
}

static const struct net_device_ops sd5116_pie_netdev_ops = {
	.ndo_open		= sd5116_pie_open,
	.ndo_stop		= sd5116_pie_stop,
	.ndo_start_xmit		= sd5116_pie_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

static int sd5116_pie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev;
	struct sd5116_pie *pie;
	int irq, ret;

	ndev = devm_alloc_etherdev(dev, sizeof(*pie));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, dev);
	pie = netdev_priv(ndev);
	pie->ndev = ndev;
	pie->dev = dev;

	pie->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pie->base))
		return PTR_ERR(pie->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = sd5116_pie_alloc_rings(pie);
	if (ret)
		return ret;

	/* the bootloader may have left sources unmasked */
	writel(PIE_IRQ_ALL, pie->base + PIE_IRQ_MASK);
	writel(PIE_IRQ_ALL, pie->base + PIE_IRQ_STATUS);

	ret = devm_request_irq(dev, irq, sd5116_pie_irq, 0, dev_name(dev), pie);
	if (ret)
		return ret;

	ret = of_get_ethdev_address(dev->of_node, ndev);
	if (ret)
		eth_hw_addr_random(ndev);

	ndev->netdev_ops = &sd5116_pie_netdev_ops;
	ndev->max_mtu = TX_BUF_SIZE - ETH_HLEN - ETH_FCS_LEN;
	netif_napi_add(ndev, &pie->napi, sd5116_pie_poll);

	return devm_register_netdev(dev, ndev);
}

static const struct of_device_id sd5116_pie_of_match[] = {
	{ .compatible = "hisilicon,sd5116-pie" },
	{ }
};
MODULE_DEVICE_TABLE(of, sd5116_pie_of_match);

static struct platform_driver sd5116_pie_driver = {
	.probe = sd5116_pie_probe,
	.driver = {
		.name = "sd5116-pie",
		.of_match_table = sd5116_pie_of_match,
	},
};
module_platform_driver(sd5116_pie_driver);

MODULE_AUTHOR("Mario Moura <mm@mariomoura.com>");
MODULE_DESCRIPTION("HiSilicon SD5116 PIE Ethernet driver");
MODULE_LICENSE("GPL");
