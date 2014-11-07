/* drivers/net/ks8851.c
 *
 * Copyright 2009 Simtec Electronics
 *	http://www.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define DEBUG

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/cache.h>
#include <linux/crc32.h>
#include <linux/if_arp.h>
#include <linux/ipv6.h>
#include <net/ip6_fib.h>

#include <linux/spi/spi.h>

#ifdef CONFIG_PLAT_STELLARIS
#  include <mach/hardware.h>
#endif
#if defined(CONFIG_MACH_ATLAS) || defined(CONFIG_MACH_UWIC)
#  include <mach/pins.h>
#endif

#include <net/omega_rf.h>
#include "spi-uip.h"

#define OMEGA_RF_HLEN		(sizeof(struct omega_rf_hdr))
#define OMEGA_RF_CMD_TIMEOUT	100

struct omega_rf_net {
	struct net_device	*netdev;
	struct spi_device	*spidev;
	struct mutex		lock;
	spinlock_t		statelock;

	u32			msg_enable ____cacheline_aligned;

	struct work_struct	tx_work;
	struct work_struct	irq_work;
	struct work_struct	reset_work;
	unsigned long		trans_start;
	int			ready_phase;

	struct sk_buff_head	txq;

	struct spi_message	spi_msg;
	struct spi_transfer	spi_xfer;

	struct spi_status_t	status;

	struct omega_rf_platform_data	*plat;
};

static int rf_is_ready(struct omega_rf_net *priv);
static int rf_wait_for_ready(struct omega_rf_net *priv, int timeout);
static int rf_ack_irq(struct omega_rf_net *priv);
static int rf_can_transmit(struct omega_rf_net *priv, struct sk_buff *skb);
static int rf_write_mac_addr(struct net_device *dev);
static int rf_reset(struct omega_rf_net *priv);
static void rf_receive(struct omega_rf_net *priv);
static void rf_transmit(struct omega_rf_net *priv, struct sk_buff *txb);

/* work queue handlers */
static void rf_reset_work(struct work_struct *work);
static void rf_irq_work(struct work_struct *work);
static void rf_tx_work(struct work_struct *work);

/* net_device_ops */
static int rf_net_open(struct net_device *dev);
static int rf_net_stop(struct net_device *dev);
static netdev_tx_t rf_start_xmit(struct sk_buff *skb, struct net_device *dev);
static void rf_set_rx_mode(struct net_device *dev);
static void rf_tx_timeout(struct net_device *dev);
static int rf_set_mac_address(struct net_device *dev, void *addr);

static int msg_enable = 31;

#define rf_info(_ks, _msg...) dev_info(&(_ks)->spidev->dev, _msg)
#define rf_warn(_ks, _msg...) dev_warn(&(_ks)->spidev->dev, _msg)
#define rf_dbg(_ks, _msg...) dev_dbg(&(_ks)->spidev->dev, _msg)
#define rf_err(_ks, _msg...) dev_err(&(_ks)->spidev->dev, _msg)

static int rf_is_ready(struct omega_rf_net *priv)
{
	return gpioread(priv->plat->nrdy_gpio, 0) == priv->ready_phase;
}

static int rf_wait_for_ready(struct omega_rf_net *priv, int timeout)
{
	int i, sleep;
	
	for (i = 0; i < 10; i++) {
		if (rf_is_ready(priv))
			return 0;
		udelay(10);
	}

	while (!rf_is_ready(priv)) {
		if (timeout > 0) {
			sleep = min(10, timeout);
			msleep(sleep);
			timeout -= sleep;
		}
		else if (timeout == 0)
			return -1;
	}

	return 0;
}

static rf_toggle_ready_phase(struct omega_rf_net *priv)
{
	if (priv->ready_phase == 0)
		priv->ready_phase = 1;
	else
		priv->ready_phase = 0;
}

static int rf_can_transmit(struct omega_rf_net *priv, struct sk_buff *skb)
{
	return rf_is_ready(priv) && priv->status.tx_queue_freespace >= 6;
}

/**
 * rf_net_open - open network device
 * @dev: The network device being opened.
 *
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device.
 */
static int rf_net_open(struct net_device *dev)
{
	struct omega_rf_net *priv = netdev_priv(dev);

	/* lock the card, even if we may not actually be doing anything
	 * else at the moment */
	mutex_lock(&priv->lock);

	netif_dbg(priv, ifup, priv->netdev, "opening\n");
	priv->ready_phase = 0;

	gpiowrite(priv->plat->nrst_gpio, 1);
	if (rf_wait_for_ready(priv, 2000)) {
		gpiowrite(priv->plat->nrst_gpio, 0);
		mutex_unlock(&priv->lock);
		return -EINVAL;
	}

	rf_write_mac_addr(dev);
	
	priv->trans_start = 0;
	netif_start_queue(priv->netdev);

	netif_dbg(priv, ifup, priv->netdev, "network device up\n");

	mutex_unlock(&priv->lock);
	return 0;
}

/**
 * rf_net_stop - close network device
 * @dev: The device being closed.
 *
 * Called to close down a network device which has been active. Cancell any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state whilst it is not being used.
 */
static int rf_net_stop(struct net_device *dev)
{
	struct omega_rf_net *priv = netdev_priv(dev);

	netif_info(priv, ifdown, dev, "shutting down\n");

	netif_stop_queue(dev);

	/* stop any outstanding work */
	flush_work(&priv->irq_work);
	flush_work(&priv->tx_work);
	flush_work(&priv->reset_work);

	mutex_lock(&priv->lock);
	gpiowrite(priv->plat->nrst_gpio, 0);
	priv->ready_phase = 0;
	mutex_unlock(&priv->lock);

	/* ensure any queued tx buffers are dumped */
	while (!skb_queue_empty(&priv->txq)) {
		struct sk_buff *txb = skb_dequeue(&priv->txq);

		netif_dbg(priv, ifdown, priv->netdev,
			  "%s: freeing txb %p\n", __func__, txb);

		dev_kfree_skb(txb);
	}

	return 0;
}

/**
 * rf_start_xmit - transmit packet
 * @skb: The buffer to transmit
 * @dev: The device used to transmit the packet.
 *
 * Called by the network layer to transmit the @skb. Queue the packet for
 * the device and schedule the necessary work to transmit the packet when
 * it is free.
 *
 * We do this to firstly avoid sleeping with the network device locked,
 * and secondly so we can round up more than one packet to transmit which
 * means we can try and avoid generating too many transmit done interrupts.
 */
static netdev_tx_t rf_start_xmit(struct sk_buff *skb,
				     struct net_device *dev)
{
	struct omega_rf_net *priv = netdev_priv(dev);
	netdev_tx_t ret = NETDEV_TX_OK;

	printk("rf_start_xmit\n");

	netif_dbg(priv, tx_queued, priv->netdev,
		  "%s: skb %p, %d@%p\n", __func__, skb, skb->len, skb->data);

	spin_lock(&priv->statelock);

	if (!rf_can_transmit(priv, skb) || (priv->trans_start && time_after(jiffies, (priv->trans_start + HZ)))) {
		printk("netif_stop_queue\n");
		netif_stop_queue(dev);
		ret = NETDEV_TX_BUSY;
	} else {
		dev->trans_start = jiffies;
		if (priv->trans_start == 0)
			priv->trans_start = jiffies;
		priv->status.tx_queue_freespace = 0;
		skb_queue_tail(&priv->txq, skb);
	}

	spin_unlock(&priv->statelock);
	schedule_work(&priv->tx_work);

	return ret;
}

static void rf_set_rx_mode(struct net_device *dev)
{
	//struct omega_rf_net *priv = netdev_priv(dev);

	if (dev->flags & IFF_PROMISC) {
		/* interface to receive everything */
	} else if (dev->flags & IFF_ALLMULTI) {
		/* accept all multicast packets */
	} else if (dev->flags & IFF_MULTICAST && dev->mc_count > 0) {
		/* accept some multicast */
	} else {
		/* just accept broadcast / unicast */
	}
}

static void rf_tx_timeout(struct net_device *dev)
{
	struct omega_rf_net *priv = netdev_priv(dev);

	dev->stats.tx_errors++;
	schedule_work(&priv->reset_work);
}

static int rf_write_mac_addr(struct net_device *dev)
{
	struct omega_rf_net *priv = netdev_priv(dev);
	struct spi_transfer *xfer = &priv->spi_xfer;
	struct spi_message *msg = &priv->spi_msg;
	struct spi_command_t cmd;
	struct spi_status_t status;
	int ret;

	if (rf_wait_for_ready(priv, OMEGA_RF_CMD_TIMEOUT)) {
		ret = -ETIMEDOUT;
		goto err;
	}

	cmd.magic = SPI_CMD_MAGIC;
	cmd.cmd = SPI_CMD_SETHWADDR;

	cmd.setaddr.addr[0] = 0x02;
	cmd.setaddr.addr[1] = 0x00;
	memcpy(&cmd.setaddr.addr[2], dev->dev_addr, ETH_ALEN);

	xfer->tx_buf = &cmd;
	xfer->rx_buf = &status;
	xfer->len = sizeof(struct spi_command_t);

	rf_toggle_ready_phase(priv);
	ret = spi_sync(priv->spidev, msg);
	if (ret < 0) {
		rf_err(priv, "%s: spi_sync() failed\n", __func__);
		goto err;
	}

	if (status.magic != SPI_STATUS_MAGIC) {
		ret = -EIO;
		goto err;
	}

	memcpy(&priv->status, &status, sizeof(struct spi_status_t));

	return 0;
	
err:
	schedule_work(&priv->reset_work);
	return ret;
}

static int rf_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *sa = addr;
	
	printk("set mac\n");

	if (netif_running(dev))
		return -EBUSY;

	if (!is_valid_ether_addr(sa->sa_data))
		return -EADDRNOTAVAIL;

	dev->addr_assign_type &= ~NET_ADDR_RANDOM;
	memcpy(dev->dev_addr, sa->sa_data, ETH_ALEN);

	return 0;
}

/**
 * rf_irq - device interrupt handler
 * @irq: Interrupt number passed from the IRQ handler.
 * @pw: The private word passed to register_irq(), our struct omega_rf_net.
 *
 * Disable the interrupt from happening again until we've processed the
 * current status by scheduling rf_irq_work().
 */
static irqreturn_t rf_irq(int irq, void *pw)
{
	struct omega_rf_net *priv = pw;

#ifdef CONFIG_PLAT_STELLARIS
	gpioclearint(GPIO_RF_INTRN);
#endif

	disable_irq_nosync(irq);
	schedule_work(&priv->irq_work);
	return IRQ_HANDLED;
}

static int rf_reset(struct omega_rf_net *priv)
{
	printk("^^^^^^^^^^^^ reset ^^^^^^^^^^^^\n");
	netif_stop_queue(priv->netdev);
	gpiowrite(priv->plat->nrst_gpio, 0);
	udelay(100);

	mutex_unlock(&priv->lock);

	/* ensure any queued tx buffers are dumped */
	while (!skb_queue_empty(&priv->txq)) {
		struct sk_buff *txb = skb_dequeue(&priv->txq);

		netif_dbg(priv, ifdown, priv->netdev,
			  "%s: freeing txb %p\n", __func__, txb);

		dev_kfree_skb(txb);
	}

	rf_net_open(priv->netdev); /* reopen device */

	mutex_lock(&priv->lock);

	return 0;
}

static void rf_reset_work(struct work_struct *work)
{
	struct omega_rf_net *priv = container_of(work, struct omega_rf_net, reset_work);

	mutex_lock(&priv->lock);

	rf_reset(priv);

	mutex_unlock(&priv->lock);
}

/**
 * rf_irq_work - work queue handler for dealing with interrupt requests
 * @work: The work structure that was scheduled by schedule_work()
 *
 * This is the handler invoked when the rf_irq() is called to find out
 * what happened, as we cannot allow ourselves to sleep whilst waiting for
 * anything other process has the chip's lock.
 *
 * Read the interrupt status, work out what needs to be done and then clear
 * any of the interrupts that are not needed.
 */
static void rf_irq_work(struct work_struct *work)
{
	struct omega_rf_net *priv = container_of(work, struct omega_rf_net, irq_work);
	int ret;

	mutex_lock(&priv->lock);

	netif_dbg(priv, intr, priv->netdev, "%s\n", __func__);
	ret = rf_ack_irq(priv);
	if (ret < 0) {
		schedule_work(&priv->reset_work);
	}
	else {
		if (ret & SPI_IRQ_RX) {
			rf_receive(priv);
			printk("rx done\n");
		}

		if (ret & SPI_IRQ_TX) {
			if (priv->status.tx_queue_freespace >= 6) {
				netif_wake_queue(priv->netdev);
			}
			priv->trans_start = 0;
			printk("tx done\n");
		}
	}

	mutex_unlock(&priv->lock);

	enable_irq(priv->netdev->irq);
}

static void rf_receive(struct omega_rf_net *priv)
{
	struct spi_transfer *xfer = &priv->spi_xfer;
	struct spi_message *msg = &priv->spi_msg;
	struct spi_command_t cmd;
	struct spi_command_t status;
	struct sk_buff *skb;
	uint8_t *rxpkt;
	uint16_t pktlen = priv->status.rx_packet_size;
	int ret;

	if (pktlen == 0)
		return;

	skb = netdev_alloc_skb(priv->netdev, pktlen + 8);
	if (skb == 0)
		return;
	skb_reserve(skb, 8);

	cmd.magic = SPI_CMD_MAGIC;
	cmd.cmd = SPI_CMD_READ;

	cmd.read.size = pktlen;

	xfer->tx_buf = &cmd;
	xfer->rx_buf = &status;
	xfer->len = sizeof(struct spi_command_t);

	rf_toggle_ready_phase(priv);
	ret = spi_sync(priv->spidev, msg);
	if (ret < 0) {
		rf_err(priv, "%s: spi_sync() failed\n", __func__);
		goto err_skb;
	}
	
	if (status.magic != SPI_STATUS_MAGIC) {
		rf_err(priv, "%s: invalid device response\n", __func__);
		goto err_skb;
	}

	if (rf_wait_for_ready(priv, OMEGA_RF_CMD_TIMEOUT)) {
		rf_err(priv, "%s: device timeout\n", __func__);
		goto err_skb;
	}
	
	rxpkt = skb_put(skb, pktlen);
	
	xfer->tx_buf = 0;
	xfer->rx_buf = rxpkt;
	xfer->len = pktlen;

	rf_toggle_ready_phase(priv);
	ret = spi_sync(priv->spidev, msg);
	if (ret < 0) {
		rf_err(priv, "%s: spi_sync() failed\n", __func__);
		goto err_skb;
	}

	printk("------------------- incoming packet -----------------\n");

	skb_pull(skb, priv->netdev->hard_header_len);
	skb_reset_network_header(skb);
	printk("ipv6hdr: %p\n", ipv6_hdr(skb));
	skb->protocol = htons(ETH_P_IPV6);
	skb->pkt_type = PACKET_HOST;
	//skb->protocol = eth_type_trans(skb, priv->netdev);

	printk("len: %u\n", skb->len);
	print_hex_dump(KERN_DEFAULT, "", DUMP_PREFIX_NONE, 16, 1, skb->data, skb->len, true);
	printk("---------------------------------------\n");

	netif_rx_ni(skb);

	priv->netdev->stats.rx_packets++;
	priv->netdev->stats.rx_bytes += pktlen;
	
	return;
	
err_skb:
	dev_kfree_skb(skb);
	schedule_work(&priv->reset_work);
}

static void rf_transmit(struct omega_rf_net *priv, struct sk_buff *txb)
{
	struct spi_transfer *xfer = &priv->spi_xfer;
	struct spi_message *msg = &priv->spi_msg;
	struct spi_command_t cmd;
	struct spi_status_t status;
	int ret;

	if (rf_wait_for_ready(priv, OMEGA_RF_CMD_TIMEOUT)) {
		goto err;
	}

	cmd.cmd = SPI_CMD_WRITE;
	cmd.write.size = txb->len;
	
	printk("rf_transmit len:%u\n", txb->len);

	xfer->tx_buf = &cmd;
	xfer->rx_buf = &status;
	xfer->len = sizeof(struct spi_command_t);

	rf_toggle_ready_phase(priv);
	ret = spi_sync(priv->spidev, msg);
	if (ret < 0) {
		rf_err(priv, "%s: spi_sync() failed\n", __func__);
		goto err;
	}

	if (rf_wait_for_ready(priv, OMEGA_RF_CMD_TIMEOUT)) {
		goto err;
	}

	xfer->tx_buf = txb->data;
	xfer->rx_buf = 0;
	xfer->len = txb->len;

	rf_toggle_ready_phase(priv);
	ret = spi_sync(priv->spidev, msg);
	if (ret < 0) {
		rf_err(priv, "%s: spi_sync() failed\n", __func__);
		goto err;
	}

	return;
	
err:
	rf_tx_timeout(priv->netdev);
}

static int rf_ack_irq(struct omega_rf_net *priv)
{
	struct spi_transfer *xfer = &priv->spi_xfer;
	struct spi_message *msg = &priv->spi_msg;
	struct spi_command_t cmd;
	struct spi_status_t status;
	int ret;
	//int i;
	
	if (rf_wait_for_ready(priv, OMEGA_RF_CMD_TIMEOUT)) {
		return -ETIMEDOUT;
	}
	
	cmd.cmd = SPI_CMD_IRQACK;

	xfer->tx_buf = &cmd;
	xfer->rx_buf = &status;
	xfer->len = sizeof(struct spi_command_t);

	rf_toggle_ready_phase(priv);
	ret = spi_sync(priv->spidev, msg);
	if (ret < 0) {
		rf_err(priv, "%s: spi_sync() failed\n", __func__);
		return ret;
	}
	
	if (status.magic != SPI_STATUS_MAGIC) {
		return -EIO;
	}

	printk("tx free: %u\n", status.tx_queue_freespace);
	//printk("status: ");
	//for (i = 0; i < 16; i++) {
	//	printk("%02X ", ((uint8_t*)&status)[i]);
	//}
	//printk("\n");

	memcpy(&priv->status, &status, sizeof(struct spi_status_t));

	return (int)status.pending_irqs;
}

/**
 * rf_tx_work - process tx packet(s)
 * @work: The work strucutre what was scheduled.
 *
 * This is called when a number of packets have been scheduled for
 * transmission and need to be sent to the device.
 */
static void rf_tx_work(struct work_struct *work)
{
	struct omega_rf_net *priv = container_of(work, struct omega_rf_net, tx_work);
	struct net_device *dev = priv->netdev;
	struct sk_buff *txb;
	bool last = skb_queue_empty(&priv->txq);

	mutex_lock(&priv->lock);

	while (!last) {
		txb = skb_dequeue(&priv->txq);
		last = skb_queue_empty(&priv->txq);

		if (txb != NULL) {
			rf_transmit(priv, txb);

			dev->stats.tx_bytes += txb->len;
			dev->stats.tx_packets++;

			dev_kfree_skb(txb);
		}
	}

	mutex_unlock(&priv->lock);
}

static const struct net_device_ops rf_netdev_ops ____cacheline_aligned = {
	.ndo_open		= rf_net_open,
	.ndo_stop		= rf_net_stop,
	.ndo_do_ioctl		= 0,
	.ndo_start_xmit		= rf_start_xmit,
	.ndo_set_mac_address	= rf_set_mac_address,
	.ndo_set_rx_mode	= rf_set_rx_mode,
	.ndo_change_mtu		= 0,
	.ndo_validate_addr	= 0,
	.ndo_tx_timeout		= rf_tx_timeout,
};



/**
 * eth_header - create the Ethernet header
 * @skb:	buffer to alter
 * @dev:	source device
 * @type:	Ethernet type field
 * @daddr: destination address (NULL leave destination address)
 * @saddr: source address (NULL use device source address)
 * @len:   packet length (<= skb->len)
 *
 *
 * Set the protocol type. For a packet of type ETH_P_802_3/2 we put the length
 * in here instead.
 */
int rf_header(struct sk_buff *skb, struct net_device *dev,
	       unsigned short type,
	       const void *daddr, const void *saddr, unsigned len)
{
	struct rt6_info *rt = (struct rt6_info*)skb_dst(skb);
	struct omega_rf_hdr *hdr = (struct omega_rf_hdr *)skb_push(skb, OMEGA_RF_HLEN);

	if (!rt)
		return -OMEGA_RF_HLEN;

	if (saddr == 0)
		saddr = dev->dev_addr;

	daddr = &rt->rt6i_gateway.in6_u.u6_addr8[10];

	hdr->srcaddr[0] = 0x02;
	hdr->srcaddr[1] = 0x00;
	memcpy(&hdr->srcaddr[2], saddr, ETH_ALEN);

	hdr->destaddr[0] = 0x02;
	hdr->destaddr[1] = 0x00;
	memcpy(&hdr->destaddr[2], daddr, ETH_ALEN);

	return OMEGA_RF_HLEN;
}

/**
 * rf_rebuild_header- rebuild the Ethernet MAC header.
 * @skb: socket buffer to update
 *
 * This is called after an ARP or IPV6 ndisc it's resolution on this
 * sk_buff. We now let protocol (ARP) fill in the other fields.
 *
 * This routine CANNOT use cached dst->neigh!
 * Really, it is used only when dst->neigh is wrong.
 */
int rf_rebuild_header(struct sk_buff *skb)
{
	return 0;
}

int rf_header_parse(const struct sk_buff *skb, unsigned char *haddr)
{
	struct omega_rf_hdr *hdr = (struct omega_rf_hdr*)skb_mac_header(skb);
	memcpy(haddr, &hdr->srcaddr[2], ETH_ALEN);
	return ETH_ALEN;
}

const struct header_ops rf_header_ops ____cacheline_aligned = {
	.create		= rf_header,
	.parse		= rf_header_parse,
	.rebuild	= rf_rebuild_header,
	.cache		= 0,
	.cache_update	= 0,
};

/* driver bus management functions */

#ifdef CONFIG_PM
static int rf_suspend(struct spi_device *spi, pm_message_t state)
{
	struct omega_rf_net *priv = dev_get_drvdata(&spi->dev);
	struct net_device *dev = priv->netdev;

	if (netif_running(dev)) {
		netif_device_detach(dev);
		rf_net_stop(dev);
	}

	return 0;
}

static int rf_resume(struct spi_device *spi)
{
	struct omega_rf_net *priv = dev_get_drvdata(&spi->dev);
	struct net_device *dev = priv->netdev;

	if (netif_running(dev)) {
		rf_net_open(dev);
		netif_device_attach(dev);
	}

	return 0;
}
#else
#define rf_suspend NULL
#define rf_resume NULL
#endif

static void rf_netdev_setup(struct net_device *dev)
{
	dev->netdev_ops = &rf_netdev_ops;
	dev->header_ops = &rf_header_ops;

	dev->type               = ARPHRD_ETHER;
	dev->hard_header_len    = OMEGA_RF_HLEN;
	dev->mtu                = ETH_DATA_LEN;
	dev->addr_len           = ETH_ALEN;
	dev->tx_queue_len       = 10;
	dev->watchdog_timeo     = HZ * 5;

	memset(dev->broadcast,0xFF, ETH_ALEN);

	dev->flags              = IFF_BROADCAST|IFF_MULTICAST|IFF_NOARP|IFF_SLAVE;
}

static int __devinit rf_probe(struct spi_device *spi)
{
	struct net_device *ndev;
	struct omega_rf_net *priv;
	struct omega_rf_platform_data *plat = (struct omega_rf_platform_data*)spi->dev.platform_data;
	int ret;

	if (!plat)
		return -EINVAL;

	ndev = alloc_netdev(sizeof(struct omega_rf_net), "rf%d", rf_netdev_setup);
	if (!ndev)
		return -ENOMEM;

	spi->bits_per_word = 8;

	priv = netdev_priv(ndev);

	priv->netdev = ndev;
	priv->spidev = spi;
	priv->trans_start = 0;
	priv->ready_phase = 0;
	priv->plat = plat;

	mutex_init(&priv->lock);
	spin_lock_init(&priv->statelock);

	INIT_WORK(&priv->tx_work, rf_tx_work);
	INIT_WORK(&priv->irq_work, rf_irq_work);
	INIT_WORK(&priv->reset_work, rf_reset_work);

	/* initialise pre-made spi transfer messages */

	spi_message_init(&priv->spi_msg);
	spi_message_add_tail(&priv->spi_xfer, &priv->spi_msg);

	dev_info(&spi->dev, "message enable is %d\n", msg_enable);

	/* set the default message enable */
	priv->msg_enable = netif_msg_init(msg_enable, (NETIF_MSG_DRV |
						     NETIF_MSG_PROBE |
						     NETIF_MSG_LINK));

	skb_queue_head_init(&priv->txq);

	SET_NETDEV_DEV(ndev, &spi->dev);

	dev_set_drvdata(&spi->dev, priv);

	ndev->irq = spi->irq;

	/* reset the device. */
	gpiowrite(plat->nrst_gpio, 0);

	ret = request_irq(spi->irq, rf_irq, IRQF_TRIGGER_LOW, ndev->name, priv);
	if (ret < 0) {
		dev_err(&spi->dev, "failed to get irq\n");
		goto err_irq;
	}

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&spi->dev, "failed to register network device\n");
		goto err_netdev;
	}

	netdev_info(ndev, "MAC %pM, IRQ %d\n", ndev->dev_addr, ndev->irq);

	return 0;


err_netdev:
	free_irq(ndev->irq, priv);

err_irq:
	free_netdev(ndev);
	return ret;
}

static int __devexit rf_remove(struct spi_device *spi)
{
	struct omega_rf_net *priv = dev_get_drvdata(&spi->dev);

	if (netif_msg_drv(priv))
		dev_info(&spi->dev, "remove\n");

	unregister_netdev(priv->netdev);
	free_irq(spi->irq, priv);
	free_netdev(priv->netdev);

	return 0;
}

static struct spi_driver rf_driver = {
	.driver = {
		.name = "omega_rf",
		.owner = THIS_MODULE,
	},
	.probe = rf_probe,
	.remove = __devexit_p(rf_remove),
	.suspend = rf_suspend,
	.resume = rf_resume,
};

static int __init rf_init(void)
{
	return spi_register_driver(&rf_driver);
}

static void __exit rf_exit(void)
{
	spi_unregister_driver(&rf_driver);
}

module_init(rf_init);
module_exit(rf_exit);

MODULE_DESCRIPTION("OmegaRF driver");
MODULE_LICENSE("GPL");

module_param_named(message, msg_enable, int, 0);
MODULE_PARM_DESC(message, "Message verbosity level (0=none, 31=all)");
MODULE_ALIAS("spi:omegarf");
