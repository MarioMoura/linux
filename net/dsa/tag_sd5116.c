// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon SD5116 switch tag support
 *
 * The switch has no CPU tag. Each user port gets a tag_8021q standalone VLAN,
 * which the switch adds on ingress of the user port, and a separate VLAN for
 * frames from the CPU, which it strips on egress of the user port.
 */

#include <linux/dsa/8021q.h>
#include <linux/dsa/sd5116.h>

#include "tag.h"
#include "tag_8021q.h"

#define SD5116_8021Q_NAME "sd5116-8021q"

static struct sk_buff *sd5116_xmit(struct sk_buff *skb,
				   struct net_device *netdev)
{
	struct dsa_port *dp = dsa_user_to_port(netdev);

	return dsa_8021q_xmit(skb, netdev, ETH_P_8021Q,
			      SD5116_TX_VID(dsa_tag_8021q_standalone_vid(dp)));
}

static struct sk_buff *sd5116_rcv(struct sk_buff *skb,
				  struct net_device *netdev)
{
	int src_port = -1, switch_id = -1, vbid = -1, vid = -1;

	dsa_8021q_rcv(skb, &src_port, &switch_id, &vbid, &vid);

	skb->dev = dsa_tag_8021q_find_user(netdev, src_port, switch_id,
					   vid, vbid);
	if (!skb->dev) {
		dev_warn_ratelimited(&netdev->dev,
				     "Couldn't decode source port\n");
		kfree_skb(skb);
		return NULL;
	}

	dsa_default_offload_fwd_mark(skb);

	return skb;
}

static const struct dsa_device_ops sd5116_8021q_netdev_ops = {
	.name			= SD5116_8021Q_NAME,
	.proto			= DSA_TAG_PROTO_SD5116_8021Q,
	.xmit			= sd5116_xmit,
	.rcv			= sd5116_rcv,
	.needed_headroom	= VLAN_HLEN,
	.promisc_on_conduit	= true,
};

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DSA tag driver for HiSilicon SD5116 switch, using VLAN");
MODULE_ALIAS_DSA_TAG_DRIVER(DSA_TAG_PROTO_SD5116_8021Q, SD5116_8021Q_NAME);

module_dsa_tag_driver(sd5116_8021q_netdev_ops);
