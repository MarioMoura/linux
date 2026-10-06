// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon SD5116 embedded switch driver
 *
 * The switch has a PON port (0), a CPU port (4) connected to the PIE DMA
 * engine, and Gigabit Ethernet ports (8 and up, one per LAN MAC). It has no
 * CPU tag, so tag_8021q VLANs identify the user ports.
 *
 * Register usage was determined by reverse engineering the vendor firmware.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dsa/8021q.h>
#include <linux/dsa/sd5116.h>
#include <linux/if_bridge.h>
#include <linux/if_vlan.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <net/dsa.h>

#define SD5116_NUM_PORTS	9
#define SD5116_CPU_PORT		4
#define SD5116_FIRST_GE_PORT	8
#define SD5116_NUM_VLANS	8

/* system controller */
#define SYSCTRL_DP_RST		0x30
#define SYSCTRL_DP_RST_MASK	(GENMASK(26, 25) | GENMASK(2, 0))

/* offsets from the switch base (0x11000000) */
#define EQM_CMD			0x0200048
#define EQM_CMD_START		BIT(31)
#define EQM_CMD_WRITE		BIT(30)
#define EQM_CMD_TYPE		GENMASK(27, 20)
#define EQM_CMD_IDX		GENMASK(15, 0)
#define EQM_RDATA(n)		(0x020004c + (n) * 4)
#define EQM_WDATA(n)		(0x020005c + (n) * 4)
#define EQM_SCHED		0x10
#define EQM_THRESH		0x11
#define EQM_DEPTH		0x13
#define EQM_CHAN		0x20
#define EQM_QBIND		0x22
#define EQM_QPRIO		0x23
#define EQM_CPU_WRR		0x50
#define QBIND_VALID		BIT(18)
#define QBIND_WEIGHT		GENMASK(17, 14)
#define QBIND_ETH		BIT(13)
#define QBIND_SCHED		GENMASK(4, 0)
#define QPRIO			GENMASK(18, 16)
#define CPU_QUEUE		0x09
#define CPU_QUEUE_PROFILE	0x1a
#define QUEUE_EN		BIT(7)

#define IFC_ACT(n)		(0x1150000 + (n) * 0x10)
#define OPP_POLICER		0x1440040
#define OPP_QMAP(row)		(0x1442800 + (row) * 0x20)

#define PA_PORT_CFG(p)		(0x10c0000 + (p) * 0x10)
#define PA_TAGGED_ACT		GENMASK(29, 28)
#define PA_PRIO_TAGGED_ACT	GENMASK(25, 24)
#define PA_UNTAGGED_ACT		GENMASK(21, 20)
#define TAG_ACT_ADD		1

#define L2_FDB_FLUSH		0x11801b0
#define L2_FDB_FLUSH_ALL	BIT(0)

#define MCC_BCT(vid)		(0x13c0000 + (vid) * 4)
#define MCC_BCT_VALID		BIT(10)
#define MCC_BCT_HEAD		GENMASK(9, 0)
#define MCC_MCG(i)		(0x13c4000 + (i) * 4)
#define MCC_MCG_NEXT		GENMASK(9, 0)
#define MCC_MCG_PORT		GENMASK(13, 10)
#define MCC_MCG_END		BIT(14)
#define MCC_MCG_VALID		BIT(25)

#define OPP_EGR_TAG(p)		(0x1400020 + (p) * 4)
#define OPP_TAGGED_ACT		GENMASK(22, 20)
#define TAG_ACT_STRIP_OUTER	1

#define VLAN_ENTRY(vid)		(0x14c0000 + (vid) * 0x10)
#define VLAN_MEMBERS		GENMASK(15, 0)
#define VLAN_KEEP_TAG		GENMASK(31, 16)
#define VLAN_VALID		BIT(0)
#define PORT_DEFAULT_TAG(p)	(0x14d7000 + (p) * 4)
#define DEFAULT_TAG_VID		GENMASK(11, 0)
#define DEFAULT_TAG_TPID	GENMASK(17, 16)
#define TPID_SEL_8021Q		1

/* GE MAC, one block per port */
#define GE_MAC(p)		(((p) - SD5116_FIRST_GE_PORT) * 0x80000)
#define GE_MODE			0x14
#define GE_MODE_SPEED		GENMASK(1, 0)
#define GE_MODE_FULL_DUPLEX	BIT(2)
#define GE_MODE_UPDATE		0x18
#define GE_MODE_UPDATE_EN	BIT(0)

struct sd5116_vlan {
	u16 vid;
	u16 members;
};

struct sd5116_switch {
	void __iomem *base;
	void __iomem *mac;
	struct regmap *sysctrl;
	struct dsa_switch ds;
	struct sd5116_vlan vlans[SD5116_NUM_VLANS];
	u16 pvid[SD5116_NUM_PORTS];
	int eqm_err;
};

static void sd5116_eqm_cmd(struct sd5116_switch *sw, u32 cmd)
{
	u32 val;

	if (readl_poll_timeout(sw->base + EQM_CMD, val, !(val & EQM_CMD_START),
			       1, 2000))
		sw->eqm_err = -ETIMEDOUT;
	writel(cmd, sw->base + EQM_CMD);
	if (readl_poll_timeout(sw->base + EQM_CMD, val, !(val & EQM_CMD_START),
			       1, 2000))
		sw->eqm_err = -ETIMEDOUT;
}

static void sd5116_eqm_write(struct sd5116_switch *sw, u32 type, u32 idx,
			     const u32 val[4])
{
	int i;

	for (i = 0; i < 4; i++)
		writel(val[i], sw->base + EQM_WDATA(i));
	sd5116_eqm_cmd(sw, EQM_CMD_START | EQM_CMD_WRITE |
		       FIELD_PREP(EQM_CMD_TYPE, type) |
		       FIELD_PREP(EQM_CMD_IDX, idx));
}

static void sd5116_eqm_read(struct sd5116_switch *sw, u32 type, u32 idx,
			    u32 val[4])
{
	int i;

	sd5116_eqm_cmd(sw, EQM_CMD_START | FIELD_PREP(EQM_CMD_TYPE, type) |
		       FIELD_PREP(EQM_CMD_IDX, idx));
	for (i = 0; i < 4; i++)
		val[i] = readl(sw->base + EQM_RDATA(i));
}

static void sd5116_bind_queues(struct sd5116_switch *sw, u32 first, int n,
			       u32 sched, u32 flags)
{
	int i;

	for (i = 0; i < n; i++) {
		u32 bind[4] = { QBIND_VALID | FIELD_PREP(QBIND_WEIGHT, i) |
				flags | FIELD_PREP(QBIND_SCHED, sched) };
		u32 prio[4] = { FIELD_PREP(QPRIO, 7 - i) };

		sd5116_eqm_write(sw, EQM_QBIND, first + i, bind);
		sd5116_eqm_write(sw, EQM_QPRIO, first + i, prio);
	}
}

/*
 * The egress queue manager (EQM) and output processor (OPP) tables are not
 * cleared at power-on and are all needed before any frame is forwarded. The
 * values are the vendor firmware's defaults.
 */
static void sd5116_queue_init(struct sd5116_switch *sw)
{
	static const u32 depth[][3] = {
		{ 0x01, 0x14500000, 0x0fa00000 }, { 0x02, 0x14500000, 0x0fa00000 },
		{ 0x03, 0x14500000, 0x0fa00000 }, { 0x04, 0x14500000, 0x0fa00000 },
		{ 0x05, 0x14500000, 0x0fa00000 }, { 0x06, 0x14500000, 0x0fa00000 },
		{ 0x07, 0x14500000, 0x0fa00000 }, { 0x08, 0x14500000, 0x0fa00000 },
		{ 0x09, 0x11940000, 0x0a8c0000 }, { 0x0a, 0x11f80000, 0x0a8c0000 },
		{ 0x0b, 0x125c0000, 0x0af00000 }, { 0x0c, 0x12c00000, 0x0af00000 },
		{ 0x0d, 0x13240000, 0x0b540000 }, { 0x0e, 0x13880000, 0x0b540000 },
		{ 0x0f, 0x13ec0000, 0x0bb80000 }, { 0x10, 0x14500000, 0x0bb80000 },
		{ 0x1a, 0x07d00000, 0x03e80000 }, { 0x1b, 0x07d00000, 0x03e80000 },
		{ 0x1c, 0x01f40000, 0x00c80000 }, { 0x1d, 0x01f40000, 0x00c80000 },
		{ 0x1e, 0x14500000, 0x0fa00000 }, { 0x1f, 0x14500000, 0x0bb80000 },
	};
	/* (row, first queue): a row maps priorities 0-7 to 8 queues */
	static const u16 qmap[][2] = {
		{ 2, 0x101 }, { 3, 0x101 }, { 4, 0x109 }, { 5, 0x109 },
		{ 8, 0x151 }, { 9, 0x151 }, { 10, 0x111 }, { 11, 0x111 },
		{ 12, 0x119 }, { 13, 0x119 }, { 14, 0x121 }, { 15, 0x121 },
		{ 16, 0x111 }, { 17, 0x119 }, { 18, 0x121 }, { 19, 0x129 },
		{ 20, 0x131 }, { 21, 0x139 }, { 22, 0x141 }, { 23, 0x149 },
	};
	static const u32 policer[] = { 0x81, 0xbf, 0x81, 0, 0x81, 0xbf, 0x81 };
	static const u32 thresh_q1[4] = { 0x003e0dac, 0x10000dac, 0x00000dac, 0x00000dac };
	static const u32 thresh_cpu[4] = { 0x035b01f4, 0x1020012c, 0x000001f4, 0x0100012c };
	static const u32 sched_eth[4] = { 0x83828110, 0x87868584, 0x00000088, 0 };
	static const u32 cpu_wrr[4] = { 0x13421342 };
	u32 v[4];
	int i, j;

	for (i = 0; i < ARRAY_SIZE(depth); i++) {
		u32 d[4] = { depth[i][1], depth[i][2], 0x1d4c0000 };

		sd5116_eqm_write(sw, EQM_DEPTH, depth[i][0], d);
	}
	sd5116_eqm_write(sw, EQM_THRESH, 0x01, thresh_q1);
	sd5116_eqm_write(sw, EQM_THRESH, CPU_QUEUE_PROFILE, thresh_cpu);
	sd5116_eqm_write(sw, EQM_SCHED, 0x3d, sched_eth);

	sd5116_bind_queues(sw, 0x51, 8, 8, QBIND_ETH);
	sd5116_bind_queues(sw, 0x01, 8, 3, QBIND_ETH);
	sd5116_bind_queues(sw, 0x11, 8, 0x10, QBIND_ETH);
	sd5116_bind_queues(sw, 0x19, 7, 0x11, QBIND_ETH);
	sd5116_eqm_write(sw, EQM_CPU_WRR, CPU_QUEUE, cpu_wrr);
	sd5116_bind_queues(sw, CPU_QUEUE, 8, SD5116_CPU_PORT, 0);

	/* enable the first CPU queue with its threshold profile */
	sd5116_eqm_read(sw, EQM_SCHED, CPU_QUEUE >> 4, v);
	v[(CPU_QUEUE & 0xc) / 4] |= (QUEUE_EN | CPU_QUEUE_PROFILE) <<
				    ((CPU_QUEUE & 3) * 8);
	sd5116_eqm_write(sw, EQM_SCHED, CPU_QUEUE >> 4, v);

	/* egress channel of a port -> its scheduler (port * 8), vendor shaping */
	for (i = 0; i < 2; i++) {
		u32 port = i ? SD5116_FIRST_GE_PORT : SD5116_CPU_PORT;
		u32 chan[4] = { port * 8, 0x00053977 };

		sd5116_eqm_write(sw, EQM_CHAN, port, chan);
	}

	for (i = 0; i < ARRAY_SIZE(qmap); i++)
		for (j = 0; j < 8; j++)
			writel(qmap[i][1] + j, sw->base + OPP_QMAP(qmap[i][0]) + j * 4);

	for (i = 0; i < ARRAY_SIZE(policer); i++)
		if (policer[i])
			writel(policer[i], sw->base + OPP_POLICER + i * 4);

	/* ingress classification actions used for LAN traffic */
	writel(0x00000800, sw->base + IFC_ACT(2));
	writel(0, sw->base + IFC_ACT(2) + 4);
	writel(0x00011000, sw->base + IFC_ACT(4));
}

static int sd5116_hw_init(struct sd5116_switch *sw)
{
	int vid;

	/* restarts the switch engines (and the PIE) */
	regmap_clear_bits(sw->sysctrl, SYSCTRL_DP_RST, SYSCTRL_DP_RST_MASK);
	usleep_range(1000, 2000);
	regmap_set_bits(sw->sysctrl, SYSCTRL_DP_RST, SYSCTRL_DP_RST_MASK);

	/*
	 * The address and flood tables are not cleared at power-on either:
	 * flush the learned addresses and let only the VLANs set up below
	 * flood, so no frame reaches a port outside its VLAN.
	 */
	writel(readl(sw->base + L2_FDB_FLUSH) | L2_FDB_FLUSH_ALL,
	       sw->base + L2_FDB_FLUSH);
	usleep_range(1000, 2000);
	for (vid = 0; vid < VLAN_N_VID; vid++)
		writel(0, sw->base + MCC_BCT(vid));

	sd5116_queue_init(sw);

	return sw->eqm_err;
}

/*
 * Flooding uses a per-VLAN linked list of destination ports in the MCC_mcg
 * table. VLAN slot n owns entries 1 + n * SD5116_NUM_PORTS onwards.
 */
static void sd5116_vlan_sync(struct sd5116_switch *sw, int slot)
{
	struct sd5116_vlan *v = &sw->vlans[slot];
	unsigned int head = 1 + slot * SD5116_NUM_PORTS, i = head;
	unsigned long members = v->members;
	int port;

	writel(FIELD_PREP(VLAN_MEMBERS, v->members) | VLAN_KEEP_TAG,
	       sw->base + VLAN_ENTRY(v->vid));
	writel(v->members ? VLAN_VALID : 0, sw->base + VLAN_ENTRY(v->vid) + 4);

	for_each_set_bit(port, &members, SD5116_NUM_PORTS) {
		u32 node = MCC_MCG_VALID | FIELD_PREP(MCC_MCG_PORT, port);

		if (port == find_last_bit(&members, SD5116_NUM_PORTS))
			node |= MCC_MCG_END;
		else
			node |= FIELD_PREP(MCC_MCG_NEXT, i + 1);
		writel(node, sw->base + MCC_MCG(i++));
	}
	writel(v->members ? MCC_BCT_VALID | FIELD_PREP(MCC_BCT_HEAD, head) : 0,
	       sw->base + MCC_BCT(v->vid));
}

static void sd5116_set_pvid(struct sd5116_switch *sw, int port, u16 vid)
{
	sw->pvid[port] = vid;
	writel(FIELD_PREP(DEFAULT_TAG_TPID, TPID_SEL_8021Q) |
	       FIELD_PREP(DEFAULT_TAG_VID, vid),
	       sw->base + PORT_DEFAULT_TAG(port));
}

static int sd5116_vlan_member(struct sd5116_switch *sw, u16 vid, int port,
			      bool member)
{
	int i, slot = -1;

	for (i = 0; i < SD5116_NUM_VLANS; i++) {
		if (sw->vlans[i].members && sw->vlans[i].vid == vid) {
			slot = i;
			break;
		}
		if (!sw->vlans[i].members && slot < 0)
			slot = i;
	}
	if (slot < 0)
		return -ENOSPC;
	if (!member && !(sw->vlans[slot].members & BIT(port)))
		return -ENOENT;

	sw->vlans[slot].vid = vid;
	if (member)
		sw->vlans[slot].members |= BIT(port);
	else
		sw->vlans[slot].members &= ~BIT(port);
	sd5116_vlan_sync(sw, slot);

	return 0;
}

/*
 * The switch floods frames back to their ingress port. So the tag_8021q VLAN
 * of a user port, its PVID, only has the CPU port as member, and the CPU
 * sends to the user port in a second VLAN that only has the user port.
 */
static int sd5116_tag_8021q_vlan_add(struct dsa_switch *ds, int port, u16 vid,
				     u16 flags)
{
	struct sd5116_switch *sw = ds->priv;
	int ret;

	/* egress untagging is per port, not per VLAN */
	if ((flags & BRIDGE_VLAN_INFO_UNTAGGED) &&
	    !(flags & BRIDGE_VLAN_INFO_PVID))
		return -EOPNOTSUPP;

	if (dsa_is_cpu_port(ds, port))
		return sd5116_vlan_member(sw, vid, port, true);

	ret = sd5116_vlan_member(sw, SD5116_TX_VID(vid), port, true);
	if (!ret && (flags & BRIDGE_VLAN_INFO_PVID))
		sd5116_set_pvid(sw, port, vid);

	return ret;
}

static int sd5116_tag_8021q_vlan_del(struct dsa_switch *ds, int port, u16 vid)
{
	struct sd5116_switch *sw = ds->priv;

	if (dsa_is_cpu_port(ds, port))
		return sd5116_vlan_member(sw, vid, port, false);

	if (sw->pvid[port] == vid)
		sd5116_set_pvid(sw, port, 0);

	return sd5116_vlan_member(sw, SD5116_TX_VID(vid), port, false);
}

static int sd5116_setup(struct dsa_switch *ds)
{
	struct sd5116_switch *sw = ds->priv;
	struct dsa_port *dp;
	int ret;

	ret = sd5116_hw_init(sw);
	if (ret)
		return dev_err_probe(ds->dev, ret, "queue manager timeout\n");

	/*
	 * User ports push the PVID tag on all ingress frames, already tagged
	 * ones included so they cannot pick another VLAN, and strip the outer
	 * tag on egress. The CPU port passes the tag through.
	 */
	dsa_switch_for_each_user_port(dp, ds) {
		writel(FIELD_PREP(PA_TAGGED_ACT, TAG_ACT_ADD) |
		       FIELD_PREP(PA_PRIO_TAGGED_ACT, TAG_ACT_ADD) |
		       FIELD_PREP(PA_UNTAGGED_ACT, TAG_ACT_ADD),
		       sw->base + PA_PORT_CFG(dp->index));
		writel(FIELD_PREP(OPP_TAGGED_ACT, TAG_ACT_STRIP_OUTER),
		       sw->base + OPP_EGR_TAG(dp->index));
	}

	rtnl_lock();
	ret = dsa_tag_8021q_register(ds, htons(ETH_P_8021Q));
	rtnl_unlock();

	return ret;
}

static void sd5116_teardown(struct dsa_switch *ds)
{
	rtnl_lock();
	dsa_tag_8021q_unregister(ds);
	rtnl_unlock();
}

static enum dsa_tag_protocol
sd5116_get_tag_protocol(struct dsa_switch *ds, int port,
			enum dsa_tag_protocol mp)
{
	return DSA_TAG_PROTO_SD5116_8021Q;
}

static void sd5116_phylink_get_caps(struct dsa_switch *ds, int port,
				    struct phylink_config *config)
{
	if (port == SD5116_CPU_PORT) {
		__set_bit(PHY_INTERFACE_MODE_INTERNAL,
			  config->supported_interfaces);
		config->mac_capabilities = MAC_1000FD;
	} else if (port >= SD5116_FIRST_GE_PORT) {
		__set_bit(PHY_INTERFACE_MODE_GMII,
			  config->supported_interfaces);
		config->mac_capabilities = MAC_10 | MAC_100 | MAC_1000;
	}
}

static void sd5116_mac_config(struct phylink_config *config, unsigned int mode,
			      const struct phylink_link_state *state)
{
}

static void sd5116_mac_link_down(struct phylink_config *config,
				 unsigned int mode, phy_interface_t interface)
{
}

static void sd5116_mac_link_up(struct phylink_config *config,
			       struct phy_device *phydev, unsigned int mode,
			       phy_interface_t interface, int speed, int duplex,
			       bool tx_pause, bool rx_pause)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct sd5116_switch *sw = dp->ds->priv;
	void __iomem *mac;
	u32 val;

	if (dp->index < SD5116_FIRST_GE_PORT)
		return;

	mac = sw->mac + GE_MAC(dp->index);
	val = FIELD_PREP(GE_MODE_SPEED, speed == SPEED_1000 ? 2 :
					speed == SPEED_100 ? 1 : 0);
	if (duplex == DUPLEX_FULL)
		val |= GE_MODE_FULL_DUPLEX;

	writel(readl(mac + GE_MODE_UPDATE) | GE_MODE_UPDATE_EN,
	       mac + GE_MODE_UPDATE);
	writel(val, mac + GE_MODE);
	writel(readl(mac + GE_MODE_UPDATE) & ~GE_MODE_UPDATE_EN,
	       mac + GE_MODE_UPDATE);
}

static const struct phylink_mac_ops sd5116_phylink_mac_ops = {
	.mac_config	= sd5116_mac_config,
	.mac_link_down	= sd5116_mac_link_down,
	.mac_link_up	= sd5116_mac_link_up,
};

static const struct dsa_switch_ops sd5116_switch_ops = {
	.get_tag_protocol	= sd5116_get_tag_protocol,
	.setup			= sd5116_setup,
	.teardown		= sd5116_teardown,
	.phylink_get_caps	= sd5116_phylink_get_caps,
	.tag_8021q_vlan_add	= sd5116_tag_8021q_vlan_add,
	.tag_8021q_vlan_del	= sd5116_tag_8021q_vlan_del,
};

static int sd5116_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sd5116_switch *sw;

	sw = devm_kzalloc(dev, sizeof(*sw), GFP_KERNEL);
	if (!sw)
		return -ENOMEM;

	sw->base = devm_platform_ioremap_resource_byname(pdev, "switch");
	if (IS_ERR(sw->base))
		return PTR_ERR(sw->base);

	sw->mac = devm_platform_ioremap_resource_byname(pdev, "mac");
	if (IS_ERR(sw->mac))
		return PTR_ERR(sw->mac);

	sw->sysctrl = syscon_regmap_lookup_by_phandle(dev->of_node,
						      "hisilicon,sysctrl");
	if (IS_ERR(sw->sysctrl))
		return dev_err_probe(dev, PTR_ERR(sw->sysctrl),
				     "failed to get system controller\n");

	sw->ds.dev = dev;
	sw->ds.num_ports = SD5116_NUM_PORTS;
	sw->ds.ops = &sd5116_switch_ops;
	sw->ds.phylink_mac_ops = &sd5116_phylink_mac_ops;
	sw->ds.priv = sw;
	platform_set_drvdata(pdev, sw);

	return dsa_register_switch(&sw->ds);
}

static void sd5116_remove(struct platform_device *pdev)
{
	struct sd5116_switch *sw = platform_get_drvdata(pdev);

	if (sw)
		dsa_unregister_switch(&sw->ds);
}

static void sd5116_shutdown(struct platform_device *pdev)
{
	struct sd5116_switch *sw = platform_get_drvdata(pdev);

	if (!sw)
		return;

	dsa_switch_shutdown(&sw->ds);
	platform_set_drvdata(pdev, NULL);
}

static const struct of_device_id sd5116_of_match[] = {
	{ .compatible = "hisilicon,sd5116-switch" },
	{ }
};
MODULE_DEVICE_TABLE(of, sd5116_of_match);

static struct platform_driver sd5116_driver = {
	.probe = sd5116_probe,
	.remove = sd5116_remove,
	.shutdown = sd5116_shutdown,
	.driver = {
		.name = "sd5116-switch",
		.of_match_table = sd5116_of_match,
	},
};
module_platform_driver(sd5116_driver);

MODULE_AUTHOR("Mario Moura <mm@mariomoura.com>");
MODULE_DESCRIPTION("HiSilicon SD5116 embedded switch driver");
MODULE_LICENSE("GPL");
