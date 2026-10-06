/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _NET_DSA_SD5116_H
#define _NET_DSA_SD5116_H

/*
 * The switch floods frames back to their ingress port, so frames from the
 * CPU use their own VLAN per user port, just below the tag_8021q range,
 * while the tag_8021q VLAN only reaches the CPU.
 */
#define SD5116_TX_VID(vid)	((vid) - 1024)

#endif
