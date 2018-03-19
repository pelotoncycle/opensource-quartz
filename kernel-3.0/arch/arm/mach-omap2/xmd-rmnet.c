/*
 * xmd-rmnet.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * Copyright (C) 2011-2012 Intel Mobile Communications GmbH
 * Author: Chaitanya <Chaitanya.Khened@intel.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * RMNET Driver modified by Intel from Google msm_rmnet.c
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/ipv6.h>
#include <linux/ip.h>
#include <asm/byteorder.h>

#include "xmd-ch.h"
#include "xmd-hsi-ll-if.h"

/* #define XMD_RMNET_ENABLE_DEBUG_MSG */
#define XMD_RMNET_ENABLE_ERR_MSG

#define RMNET_IPV6_VER 0x6
#define RMNET_IPV4_VER 0x4

#define RMNET_WD_TMO      20
#define MAX_PART_PKT_SIZE 2500

#ifdef XMD_RMNET_ENABLE_DEBUG_MSG
#define xmd_rmnet_dbg(fmt, args...)  printk("\nrmnet: " fmt "\n", ## args)
#else
#define xmd_rmnet_dbg(fmt, args...)  do { } while (0)
#endif

#ifdef XMD_RMNET_ENABLE_ERR_MSG
#define xmd_rmnet_err(fmt, args...)  printk("\nrmnet: " fmt "\n", ## args)
#else
#define xmd_rmnet_err(fmt, args...)  do { } while (0)
#endif

#if !defined (CONFIG_EN_HSI_EDLP)
typedef enum {
	RMNET_FULL_PACKET,
	RMNET_PARTIAL_PACKET,
	RMNET_PARTIAL_HEADER,
} RMNET_PAST_STATE;

static struct {
	RMNET_PAST_STATE state;
	char buf[MAX_PART_PKT_SIZE];
	int size;
	int type;
} past_packet[XMD_MAX_NET];
#endif

static struct xmd_ch_info rmnet_channels[XMD_MAX_NET];

extern void xmd_restart_tx(u32 chno);

/* #define USE_PRIVATE_STATS */

struct rmnet_private {
	struct xmd_ch_info *ch;
#ifdef USE_PRIVATE_STATS
	struct net_device_stats stats;
#endif
	const char *chname;
	struct wake_lock wake_lock;
};

static struct {
	int blocked;
	struct net_device *dev;
} rmnet_ch_block_info[HSI_LL_MAX_CHANNELS];

static void xmd_net_dump_packet(u32 chno, void *ptr)
{
	u32 *str = (u32*) kzalloc(256, GFP_ATOMIC);
	if(str != NULL)
	{
		u32 x=0;
		memcpy(str, ptr, 256);
		xmd_rmnet_err("ch(%d): 1st 64 words of PDU", chno);
		for(x=0; x < 64; x+=4) {
			printk("\n0x%08X\t0x%08X\t0x%08X\t0x%08X",
					str[x],str[x+1],str[x+2],str[x+3]);
		}
		kfree(str);
	}
	return;
}

#if !defined (CONFIG_EN_HSI_EDLP)
static void rmnet_reset_pastpacket_info(int ch)
{
	if(ch >= XMD_MAX_NET ) {
		xmd_rmnet_dbg("Invalid rmnet channel number %d.", ch);
		return;
	}
	past_packet[ch].state = RMNET_FULL_PACKET;
	memset(past_packet[ch].buf, 0 , MAX_PART_PKT_SIZE);
	past_packet[ch].size = 0;
	past_packet[ch].type = 0;
}
#endif

/*give the packet to TCP/IP*/
static void xmd_trans_packet(
	struct net_device *dev,
	int type,
	void *buf,
	int sz)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct sk_buff *skb;
	void *ptr = NULL;
	sz += ETH_HLEN;

	xmd_rmnet_dbg("%d <",sz);

	if (sz > 1514 /*ETH_FRAME_LEN*/) {
		xmd_rmnet_err("Discarding pkt, pkt sz %d greater than %d.",
						sz, ETH_FRAME_LEN);
		return;
	} else {
		skb = dev_alloc_skb(sz + NET_IP_ALIGN);
		if (skb == NULL) {
			xmd_rmnet_err("Cannot allocate skb, packet dropped");
			return;
		} else {
			skb->dev = dev;
			skb_reserve(skb, NET_IP_ALIGN);
			ptr = skb_put(skb, sz);
			wake_lock_timeout(&p->wake_lock, HZ / 2);
			/* adding ethernet header */
			{
				char temp[] = {0xB6,0x91,0x24,0xa8,0x14,0x72,0xb6,0x91,0x24,
							   0xa8,0x14,0x72,0x08,0x0};
				struct ethhdr *eth_hdr = (struct ethhdr *) temp;

				if (type == RMNET_IPV6_VER) {
					eth_hdr->h_proto = htons(ETH_P_IPV6);
				}

				memcpy ((void *)eth_hdr->h_dest,
						(void*)dev->dev_addr,
						sizeof(eth_hdr->h_dest));
				memcpy ((void *)ptr,
						(void *)eth_hdr,
						sizeof(struct ethhdr));
			}
			memcpy(ptr + ETH_HLEN, buf, sz - ETH_HLEN);
			skb->protocol = eth_type_trans(skb, dev);
#ifdef USE_PRIVATE_STATS
			p->stats.rx_packets++;
			p->stats.rx_bytes += skb->len;
#else
			dev->stats.rx_packets++;
			dev->stats.rx_bytes += skb->len;
#endif

			if (in_interrupt()) {
				if (netif_rx(skb)) {
					dev_kfree_skb_irq(skb);
					xmd_rmnet_err("Rx packet dropped");
				}
			} else {
				if(netif_rx_ni(skb)) {
					dev_kfree_skb_any(skb);
					xmd_rmnet_err("Rx packet dropped");
				}
			}
			wake_unlock(&p->wake_lock);
		}
	}
}

/* Called in wq context */
#if defined (CONFIG_EN_HSI_EDLP)
static void xmd_net_notify(int chno, void * arg)
{
	u32 idx;
	u32 no_of_packets = 0;
	int ver = 0;
	int data_sz = 0;
	char *ip_buf = NULL;
	char *dl_buf = NULL;
	struct net_device *dev = NULL;
	struct rmnet_private *p = NULL;
	struct xmd_ch_info *info = NULL;
	struct hsi_ll_rx_tx_data *rx_buf = (struct hsi_ll_rx_tx_data*)arg;

	for (idx=0; idx < XMD_MAX_NET; idx++) {
		if (rmnet_channels[idx].chno == chno) {
			dev = (struct net_device *)rmnet_channels[idx].priv;
			break;
		}
	}

	if (!dev) {
		xmd_rmnet_err("No device.");
		return;
	}

	p = netdev_priv(dev);
	if (!p)
		return;

	info = p->ch;
	if (!info)
		return;

	dl_buf = rx_buf->ptr;

	if((*((u32*)dl_buf) & 0xFFFF0000) != HSI_LL_PDU_SIGNATURE) {
		xmd_rmnet_err("Received Invalid packet on Ch %d", info->id);
		xmd_net_dump_packet(info->id, dl_buf);
		return; /*Do not process further*/
	}

	idx = 2; /*Signature and IDX fields to be skipped*/
	while (*((u32*)dl_buf + idx) & 0x8000000) {
		idx += 2;
	}

	no_of_packets = idx >> 1;
	idx = 4;

	while (no_of_packets) {
		ip_buf = ((char*)(dl_buf[idx]+dl_buf+16));
#if defined(__BIG_ENDIAN_BITFIELD)
		ver = ((char *)ip_buf)[0] & 0x0F;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
		ver = (((char *)ip_buf)[0] & 0xF0) >> 4;
#endif
		if (ver == RMNET_IPV4_VER) {
			data_sz = ntohs(((struct iphdr*) ip_buf)->tot_len);
		} else if (ver == RMNET_IPV6_VER) {
			data_sz = ntohs(((struct ipv6hdr*) ip_buf)->payload_len +
							sizeof(struct ipv6hdr));
		} else {
			xmd_rmnet_err(" rmnet(%d), Invalid version = %d. "
							"Dropping current Packet.", info->id, ver);
			xmd_net_dump_packet(info->id, dl_buf);
			return; /*Do not process further*/
		}
		/*Note: For eDLP case always a full ip packet is expected. */
		xmd_trans_packet(dev, ver, ip_buf, data_sz);
		no_of_packets--;
		idx += 8;
	}
}
#else
static void xmd_net_notify(int chno, void* arg)
{
	int i;
	struct net_device *dev = NULL;
	void *buf = NULL;
	int tot_sz = 0;
	struct rmnet_private *p = NULL;
	struct xmd_ch_info *info = NULL;
	struct hsi_ll_rx_tx_data *rx_buf = (struct hsi_ll_rx_tx_data*)arg;

	for (i=0; i < XMD_MAX_NET; i++) {
		if (rmnet_channels[i].chno == chno) {
			dev = (struct net_device *)rmnet_channels[i].priv;
			break;
		}
	}

	if (!dev) {
		xmd_rmnet_err("No device.");
		return;
	}

	p = netdev_priv(dev);
	if (!p)
		return;

	info = p->ch;
	if (!info)
		return;

	buf    = rx_buf->ptr;
	tot_sz = rx_buf->size;

	if (!buf) {
		xmd_rmnet_err("No buf recvd from ch:%d", info->chno);
		return;
	}
	xmd_rmnet_dbg("Total size read = %d from ch:%d",
				  tot_sz, info->chno);

	switch (past_packet[info->id].state)
	{
	case RMNET_FULL_PACKET:
		/* no need to do anything */
	break;

	case RMNET_PARTIAL_PACKET:
	{
		void *ip_hdr = (void *)past_packet[info->id].buf;
		int sz;
		int copy_size;

		xmd_rmnet_dbg("Past partial packet.");
		if (past_packet[info->id].type == RMNET_IPV4_VER) {
			sz = ntohs(((struct iphdr*) ip_hdr)->tot_len);
		} else if (past_packet[info->id].type == RMNET_IPV6_VER) {
			sz = ntohs(((struct ipv6hdr*) ip_hdr)->payload_len +
						sizeof(struct ipv6hdr));
		} else {
			xmd_rmnet_err("Invalid past version (data), %d",
						  past_packet[info->id].type);
			past_packet[info->id].state = RMNET_FULL_PACKET;
			return;
		}

		copy_size = sz - past_packet[info->id].size;

		 /* if read size if > then copy size, copy full packet.*/
		if (tot_sz >= copy_size) {
			memcpy (past_packet[info->id].buf + past_packet[info->id].size,
					buf,
					copy_size);
		} else {
			/* copy whatever read if read size < packet size.*/
			memcpy (past_packet[info->id].buf + past_packet[info->id].size,
					buf,
					tot_sz);
			xmd_rmnet_dbg("RMNET_PARTIAL_PACKET. past size = %d,"
					" total size = %d", past_packet[info->id].size, tot_sz);
			past_packet[info->id].size += tot_sz;
			return;
		}

		xmd_trans_packet(dev,past_packet[info->id].type,
						(void*)past_packet[info->id].buf,sz);
		xmd_rmnet_dbg("Pushed reassembled data packet to tcpip,"
					  " sz = %d", sz);
		buf = buf + copy_size;
		tot_sz = tot_sz - copy_size;
	}
	break;

	case RMNET_PARTIAL_HEADER:
	{
		void *ip_hdr = (void *)past_packet[info->id].buf;
		int sz;
		int copy_size;
		int hdr_size = 0;

		xmd_rmnet_dbg("Past partial header packet.");
		if (past_packet[info->id].type == RMNET_IPV4_VER) {
			hdr_size = sizeof(struct iphdr);
		} else if (past_packet[info->id].type  == RMNET_IPV6_VER) {
			hdr_size = sizeof(struct ipv6hdr);
		} else {
			xmd_rmnet_err("Invalid past version (hdr), %d",
						  past_packet[info->id].type);
			rmnet_reset_pastpacket_info(info->id);
			break;
		}

		copy_size = hdr_size - past_packet[info->id].size;

		if(tot_sz >= copy_size) {
			memcpy (past_packet[info->id].buf + past_packet[info->id].size,
					buf,
					copy_size);
		} else {
			/* copy whatever read if read size < packet size. */
			memcpy (past_packet[info->id].buf + past_packet[info->id].size,
					buf,
					tot_sz);
			xmd_rmnet_dbg("Still partial header.");
			past_packet[info->id].size += tot_sz;
			return;
		}

		buf = buf + copy_size;
		tot_sz = tot_sz - copy_size;
		past_packet[info->id].size = past_packet[info->id].size + copy_size;

		if (past_packet[info->id].type == RMNET_IPV4_VER) {
			sz = ntohs(((struct iphdr*) ip_hdr)->tot_len);
		} else if (past_packet[info->id].type == RMNET_IPV6_VER) {
			sz = ntohs(((struct ipv6hdr*) ip_hdr)->payload_len +
						sizeof(struct ipv6hdr));
		} else {
			xmd_rmnet_err("Invalid past version, %d",
						  past_packet[info->id].type);
			past_packet[info->id].state = RMNET_FULL_PACKET;
			return;
		}

		copy_size = sz - past_packet[info->id].size;

		 /* if read size if > then copy size, copy full packet. */
		if (tot_sz >= copy_size) {
			memcpy (past_packet[info->id].buf + past_packet[info->id].size,
					buf,
					copy_size);
		} else {
			/* copy whatever read if read size < packet size.*/
			memcpy (past_packet[info->id].buf + past_packet[info->id].size,
					buf,
					tot_sz);
			xmd_rmnet_dbg("RMNET_PARTIAL_HEADER. past size = %d,"
					" total size = %d", past_packet[info->id].size, tot_sz);
			past_packet[info->id].size += tot_sz;
			past_packet[info->id].state = RMNET_PARTIAL_PACKET;
			return;
		}

		xmd_trans_packet(dev,
						 past_packet[info->id].type,
						(void *)past_packet[info->id].buf,sz);

		buf = buf + copy_size;
		tot_sz = tot_sz - copy_size;

	}
	break;

	default:
		xmd_rmnet_err("Invalid past state %d",
					  (int)past_packet[info->id].state);
		past_packet[info->id].state = RMNET_FULL_PACKET;
		break;
	}

	while (tot_sz) {
		int hdr_size = 0;
		int ver = 0;
		void *ip_hdr = (void *)buf;
		int data_sz = 0;

#if defined(__BIG_ENDIAN_BITFIELD)
		ver = ((char *)buf)[0] & 0x0F;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
		ver = (((char *)buf)[0] & 0xF0) >> 4;
#endif

		xmd_rmnet_dbg("Ver = 0x%x, total size : %d",
						ver, tot_sz);

		if (ver == RMNET_IPV4_VER) {
			hdr_size = sizeof(struct iphdr);
		} else if (ver == RMNET_IPV6_VER) {
			hdr_size = sizeof(struct ipv6hdr);
		} else {
#if defined (XMD_RMNET_ENABLE_ERR_MSG)
			void *ip_hdr = (char*)(buf+4);
			int tmp_sz = ntohs(((struct iphdr*) ip_hdr)->tot_len);

			if (strnstr((char *)buf, "+PBREADY", 12) != NULL) {
				xmd_rmnet_err("ch %d:Ignore recieved \"+PBREADY\"", chno);
				break;
			}

			xmd_rmnet_err("Invalid version, 0x%x.", ver);
			xmd_net_dump_packet(info->id, buf);
			xmd_rmnet_err("Current packet size = %d,"
						  "ip packet size = %d",tot_sz,tmp_sz);
#endif
			break;
		}

		if (tot_sz < hdr_size) {
			past_packet[info->id].state = RMNET_PARTIAL_HEADER;
			past_packet[info->id].size = tot_sz;
			memcpy(past_packet[info->id].buf, buf, tot_sz);
			past_packet[info->id].type = ver;
			xmd_rmnet_dbg("Partial header packet copied locally,"
						  " sz = %d", tot_sz);
			return;
		}

		if (ver == RMNET_IPV4_VER) {
			data_sz = ntohs(((struct iphdr*) ip_hdr)->tot_len);
		} else if (ver == RMNET_IPV6_VER) {
			data_sz = ntohs(((struct ipv6hdr*) ip_hdr)->payload_len) +
							sizeof(struct ipv6hdr);
		} else {
			xmd_rmnet_err("Data sz check - Invalid version = %d.",ver);
			break;
		}

		xmd_rmnet_dbg("Data size = %d", data_sz);

		if (tot_sz < data_sz) {
			past_packet[info->id].state = RMNET_PARTIAL_PACKET;
			past_packet[info->id].size = tot_sz;
			memcpy(past_packet[info->id].buf, buf, tot_sz);
			past_packet[info->id].type = ver;
			xmd_rmnet_dbg("Partial data packet copied locally,"
						  " sz = %d", tot_sz);
			return;
		}

		xmd_trans_packet(dev, ver, buf, data_sz);
		xmd_rmnet_dbg("Pushed full data packet to tcpip, "
					  "sz = %d", data_sz);
		tot_sz = tot_sz - data_sz;
		buf = buf + data_sz;
		xmd_rmnet_dbg("Looping for another packet"
					  " tot_sz = %d", tot_sz);
	}

	past_packet[info->id].state = RMNET_FULL_PACKET;
}
#endif

static int rmnet_open(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct xmd_ch_info *ch = p->ch;

	pr_info("rmnet_open(%d)\n", ch->id);

	if (p->ch) {
		ch->chno = xmd_ch_open(ch, xmd_net_notify);

		if (ch->chno < 0) {
			xmd_rmnet_err("Error opening rmnet(%d)", ch->id);
			return -ENODEV;
		}
	}

	netif_start_queue(dev);
	return 0;
}

static int rmnet_stop(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct xmd_ch_info *info = p->ch;

	pr_info("rmnet_stop(%d)\n", info->id);
	netif_stop_queue(dev);
	xmd_ch_close(info->chno);
#if !defined (CONFIG_EN_HSI_EDLP)
	rmnet_reset_pastpacket_info(info->id);
#endif
	return 0;
}

void rmnet_restart_queue(int chno)
{
	if(rmnet_ch_block_info[chno].blocked) {
		rmnet_ch_block_info[chno].blocked = 0;
		netif_wake_queue(rmnet_ch_block_info[chno].dev);
		xmd_rmnet_dbg("Queue free so unblocking rmnet(%d) queue.", chno);
	}
}

static int rmnet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct xmd_ch_info *info = p->ch;
	int ret;
	void *ptr;
	int sz = 0;

	xmd_rmnet_dbg("rmnet(%d): %d>", info->chno, skb->len);
	if((skb->len - ETH_HLEN) <= 0) {
		xmd_rmnet_dbg("Got only header for ch %d, return", info->chno);
		ret = NETDEV_TX_OK;
		dev_kfree_skb_irq(skb);
		goto quit_xmit;
	}

#if defined (CONFIG_EN_HSI_EDLP)
	ptr = skb;
	sz = skb->len;
#else
	ptr = (void *)((char *) skb->data + ETH_HLEN);
	sz = skb->len - ETH_HLEN;
#endif
	if ((ret = xmd_ch_write(info->chno,
							ptr,
							sz)) != 0) {
		if(ret == -ENOMEM) {
			ret = NETDEV_TX_BUSY;
			xmd_rmnet_dbg("Cannot alloc mem, so returning busy for ch %d",
						  info->chno);
			goto quit_xmit;
		} else if(ret == -EBUSY) {
			netif_stop_queue(dev);
			rmnet_ch_block_info[info->chno].dev = dev;
			rmnet_ch_block_info[info->chno].blocked = 1;
			xmd_rmnet_dbg("Stopping queue for ch %d", info->chno);
			ret = NETDEV_TX_BUSY;
			goto quit_xmit;
		} else {
#if 0
			xmd_rmnet_err("xmd_ch_write error for ch %d\n", info->chno);
#else
			xmd_rmnet_err("xmd_ch_write error(%d) for ch %d\n", ret, info->chno);
#endif
		}
	} else {
#ifdef USE_PRIVATE_STATS
		p->stats.tx_packets++;
		p->stats.tx_bytes += skb->len;
#else
		dev->stats.tx_packets++;
		dev->stats.tx_bytes += skb->len;
#endif
	}
	ret = NETDEV_TX_OK;
#if !defined (CONFIG_EN_HSI_EDLP)
	dev_kfree_skb_irq(skb); /*Note: For eDLP case skb is freed in Link Layer*/
#endif
quit_xmit:
	return ret;
}

#ifdef USE_PRIVATE_STATS
static struct net_device_stats *rmnet_get_stats(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	return &p->stats;
}
#endif

static int rmnet_change_mtu(struct net_device *dev, int new_mtu)
{
	if ((ETH_HLEN >= new_mtu) || ((ETH_HLEN + 1500) < new_mtu)) {
		xmd_rmnet_err("new mtu sz %d not acceptable", new_mtu);
		return -EINVAL;
	}

	xmd_rmnet_dbg("mtu changed from %d to %d", dev->mtu, new_mtu);
	dev->mtu = new_mtu;

	return 0;
}

static void rmnet_tx_timeout(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct xmd_ch_info *info = p->ch;
	pr_info("rmnet_tx_timeout(%d)", info->id);
#if defined (CONFIG_EN_HSI_EDLP)
	xmd_restart_tx(info->chno);
#endif
}

static struct net_device_ops rmnet_ops = {
	.ndo_open = rmnet_open,
	.ndo_stop = rmnet_stop,
	.ndo_start_xmit = rmnet_xmit,
#ifdef USE_PRIVATE_STATS
	.ndo_get_stats = rmnet_get_stats,
#endif
	.ndo_tx_timeout = rmnet_tx_timeout,
	.ndo_change_mtu = rmnet_change_mtu,
};

static void __init rmnet_setup(struct net_device *dev)
{
	dev->netdev_ops = &rmnet_ops;

	dev->watchdog_timeo = RMNET_WD_TMO;
	ether_setup(dev);
	dev->flags = IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;
	random_ether_addr(dev->dev_addr);
}

static int __init rmnet_init(void)
{
	int ret;
	struct device *d;
	struct net_device *dev;
	struct rmnet_private *p;
	unsigned n;

	for (n = 0; n < XMD_MAX_NET; n++) {
		rmnet_channels[n].id = n;
		HSI_CHANNEL_NAME(rmnet_channels[n].name, (n+XMD_RMNET_HSI_CH));
		rmnet_channels[n].chno = 0;
		rmnet_channels[n].user = XMD_NET;
		rmnet_channels[n].priv = NULL;
		rmnet_channels[n].open_count = 0;
#if !defined (CONFIG_EN_HSI_EDLP)
		rmnet_reset_pastpacket_info(n);
#endif
		dev = alloc_netdev(sizeof(struct rmnet_private),
							"rmnet%d", rmnet_setup);

		if (!dev)
			return -ENOMEM;

		d = &(dev->dev);
		p = netdev_priv(dev);
		rmnet_channels[n].priv = (void *)dev;
		p->ch = rmnet_channels + n;
		p->chname = rmnet_channels[n].name;
		wake_lock_init(&p->wake_lock,
						WAKE_LOCK_SUSPEND,
						rmnet_channels[n].name);
		ret = register_netdev(dev);
		if (ret) {
			free_netdev(dev);
			return ret;
		}
	}

	return 0;
}

module_init(rmnet_init);
