/*
 * The data flow of the devices is:
 *
 *          vnet
 *     _______|___________
 *     |                  |
 *     |                  |
 *   vd_rx            vd_tx
 *   (recieve)          (transmit)
 *
 * vnet: pseodu network device
 * vd_rx: character device
 * vd_tx:  character device
 * You can modify and distribute this source code freely.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>

#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/types.h>

#include <linux/in.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/cdev.h>
#include <linux/ip.h>
#include <linux/skbuff.h>
#include <linux/ioctl.h>
#include <linux/semaphore.h>

#include <asm/uaccess.h>

#include "vd_device.h"
#include "vd_ioctl.h"

MODULE_AUTHOR("Lynx Zhou");
MODULE_LICENSE("Dual BSD/GPL");

char vd_names[16];
struct vd_device vd[2];
struct net_device *vnet;
struct net_device_ops vnet_device_ops;

//static int timeout = VD_TIMEOUT;

void vnet_rx(struct net_device *dev,int len,unsigned char *buf);

void vnet_tx_timeout (struct net_device *dev);


static int vcdev_devno_init(struct vd_device *dev, int major, int minor, char *name)
{
    int devno;
    int err;

    if (major) {
        devno = MKDEV(major, minor);
        err = register_chrdev_region(devno, 1, name);
    } else {
        err = alloc_chrdev_region(&devno, minor, 1, name);
        dev->major = MAJOR(devno);
    }

    if (err < 0) {
        printk(KERN_WARNING "vcdev: can't get major %d\n", dev->major);
        return err;
    }

    return 0;
}

static void vcdev_release(void)
{
    int devno;
    int i;
    struct vd_device* vdev;

    for (i = 0 ;i < 2; i++){
        vdev = &vd[i];
        kfree(vdev->buffer);

        devno = MKDEV(vdev->major, i);
        unregister_chrdev_region(devno, 1);
        cdev_del(&vdev->vcdev);
    }
}

/* Initialize the vd_rx and vd_tx device,the two devices
 * are allocate the initial buffer to store the incoming and
 * outgoing data. If the TCP/IP handshake need change the
 * MTU,we must reallocte the buffer using the new MTU value.
 */
static int vcdev_init(void)
{
    int i;
    int err = 0;
    struct vd_device *vdev;

    memset(vd, 0, sizeof(vd));

    strcpy(vd[VD_RX_DEVICE].name, VD_RX_DEVICE_NAME);
    strcpy(vd[VD_TX_DEVICE].name, VD_TX_DEVICE_NAME);

    for (i = 0; i < 2; i++) {
        vdev = &vd[i];
        err = vcdev_devno_init(vdev, 0, i, vdev->name);
        if (err)
            goto err_exit;
    }

    for (i = 0 ;i < 2; i++ ) {
        vdev = &vd[i];
        vdev->buffer_size = BUFFER_SIZE;
        vdev->buffer = kmalloc(vdev->buffer_size + 4 , GFP_KERNEL);
        vdev->magic = VD_MAGIC;
        vdev->mtu = VD_MTU;
        vdev->busy = 0;

        if (vdev->buffer == NULL) {
            err = -ENOBUFS;
            printk("There is no enongh memory for buffer allocation. \n");
            goto err_exit;
        }

        if (VD_TX_DEVICE == i)
            sema_init(&vdev->sem, 0);
        else
            sema_init(&vdev->sem, 1);
    }

    return 0;

err_exit:
    vcdev_release();
    return err;
}

static int vd_realloc(int new_mtu)
{
    int err = -ENOBUFS;
    int i;
    char *local_buffer[2];
    int size;

    for (i = 0; i < 2; i++){
        local_buffer[i] = kmalloc(new_mtu + 4, GFP_KERNEL);
        size = min(new_mtu, vd[i].buffer_size);

        memcpy(local_buffer[i], vd[i].buffer, size);
        kfree(vd[i].buffer);

        vd[i].buffer = kmalloc(new_mtu + 4, GFP_KERNEL);
        if(vd[i].buffer < 0){
            printk("Can not realloc the buffer from kernel when change mtu.\n");
            return err;
        }

    }

    return 0;
}

static int device_open(struct inode *inode, struct file *filp)
{
    int device_major;
    struct vd_device *vdp;

    device_major = imajor(inode);

#ifdef _DEBUG
    printk("Get the device major number is %d\n", device_major);
#endif

    vdp = container_of(inode->i_cdev, struct vd_device, vcdev);
    filp->private_data = vdp;

    if (device_major != vdp->major)
        return -ENODEV;

    if (vdp->busy != 0) {
       printk("The device is open!\n");
       return -EBUSY;
    }

    vdp->busy++;

    return 0;
}

int device_release(struct inode *inode, struct file *filp)
{
    struct vd_device *vdp;

    vdp = (struct vd_device *)filp->private_data;
    vdp->busy = 0;

    return 0;
}

/* read data from vd_tx device */
ssize_t device_read(struct file *filp, char *buffer, size_t length, loff_t *offset)
{
    #ifdef _DEBUG
    int i;
    #endif
    struct vd_device *vdp;

    vdp = (struct vd_device *)filp->private_data;

    if ((filp->f_flags & O_NONBLOCK)) {
        if (down_trylock(&vdp->sem)) {
            return -EAGAIN;
        }
    } else if (down_interruptible(&vdp->sem)) {
        return -ERESTARTSYS;
    }

    if (vdp->tx_len == 0) {
        return -EAGAIN;
    }

    #ifdef _DEBUG
    printk("\n read data from vd_tx \n");
    for (i = 0; i < vdp->tx_len; i++)
        printk(" %02x", vdp->buffer[i] & 0xff);
    printk("\n");
    #endif

    if (copy_to_user(buffer, vdp->buffer, vdp->tx_len)) {
        return -EFAULT;
    }

    memset(vdp->buffer, 0, vdp->buffer_size); // TODO: really needed?
    length = vdp->tx_len;  // TODO:
    vdp->tx_len = 0;

    return length;
}

ssize_t serial_buffer_write(const char *buffer, size_t length, int buffer_size)
{
    struct vd_device *vdp = &vd[VD_TX_DEVICE];

    if (length > buffer_size)
        length = buffer_size;

    memset(vdp->buffer, 0, buffer_size);
    memcpy(vdp->buffer, buffer, buffer_size);
    vdp->tx_len = length;

    up(&vdp->sem);

    return length;
}

/* Device write is called by server program, to put the user space
 * network data into vd_rx device.
 */
ssize_t device_write(struct file *filp, const char *buffer, size_t length, loff_t *offset)
{
    #ifdef _DEBUG
    int i;
    #endif
    struct vd_device *vdp;
    vdp = (struct vd_device *)filp->private_data;

    //spin_lock(&vd[VD_RX_DEVICE].lock);
    if (down_interruptible(&vdp->sem))
        return -ERESTARTSYS;

    if (length > vdp->buffer_size)
        length = vdp->buffer_size;

    copy_from_user(vd[VD_RX_DEVICE].buffer, buffer, length);
    vnet_rx(vnet, length, vd[VD_RX_DEVICE].buffer);

    #ifdef _DEBUG
    printk("\nNetwork Device Recieve buffer:\n");
    for (i = 0; i < length; i++)
       printk(" %02x", vd[VD_RX_DEVICE].buffer[i] & 0xff);
    printk("\n");
    #endif
    //spin_unlock(&vd[VD_RX_DEVICE].lock);
    up(&vdp->sem);

    return length;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 35)
long device_ioctl(
#else
int device_ioctl(struct inode *inode,
#endif
            struct file *filp,
            unsigned int ioctl_num,
            unsigned long ioctl_param)
{
    struct vd_device *vdp;

    vdp = (struct vd_device *)filp->private_data;

    switch(ioctl_num) {
        case IOCTL_SET_BUSY:
           vdp->busy = ioctl_param;
           break;
    }

    return 0;
}

/*
 * All the vnet_* functions are for the vnet pseudo network device vnet.
 * vnet_open and vnet_stop are the two functions which open and release
 * the device.
 */
int vnet_open(struct net_device *dev)
{
    try_module_get(THIS_MODULE);

    /* Assign the hardware pseudo network hardware address,
     * the MAC address's first octet is 00,for the MAC is
     * used for local net,not for the Internet.
     */
    memcpy(dev->dev_addr, "\0ED000", ETH_ALEN);

    netif_start_queue(dev);

    return 0;
}

int vnet_stop(struct net_device *dev)
{
    netif_stop_queue(dev);

    module_put(THIS_MODULE);

    return 0;
}

/*
 * vnet_rx,recieves a network packet and put the packet into TCP/IP up
 * layer,netif_rx() is the kernel API to do such thing. The recieving
 * procedure must alloc the sk_buff structure to store the data,
 * and the sk_buff will be freed in the up layer.
 */
void vnet_rx(struct net_device *dev, int len, unsigned char *buf)
{
    struct sk_buff *skb;
    struct vnet_priv *priv = (struct vnet_priv *)netdev_priv(dev);

    skb = dev_alloc_skb(len+2);
    if (!skb) {
        printk("vnet_rx can not allocate more memory to store the packet. drop the packet\n");
        priv->stats.rx_dropped++;
        return;
    }
    skb_reserve(skb, 2);
    memcpy(skb_put(skb, len), buf, len);

    skb->dev = dev;
    skb->protocol = eth_type_trans(skb, dev);
    /* We need not check the checksum */
    skb->ip_summed = CHECKSUM_UNNECESSARY;
    priv->stats.rx_packets++;
    priv->stats.rx_bytes += len;
    netif_rx(skb);

    return;
}

/*
 * pseudo network hareware transmit,it just put the data into the
 * vd_tx device.
 */
void vnet_hw_tx(char *buf, int len, struct net_device *dev)
{
    struct vnet_priv *priv = (struct vnet_priv *)netdev_priv(dev);

    /* check the ip packet length,it must more then 34 octets */
    if (len < sizeof(struct ethhdr) + sizeof(struct iphdr)) {
        printk("Bad packet! It's size is less then 34!\n");
        return;
    }
    /* now push the data into vd_tx device */
    vd[VD_TX_DEVICE].buffer_write(buf,len,vd[VD_TX_DEVICE].buffer_size);

    /* record the transmitted packet status */
    priv->stats.tx_packets++;
    priv->stats.rx_bytes += len;

    /* remember to free the sk_buffer allocated in upper layer. */
    dev_kfree_skb(priv->skb);
}

/*
 * Transmit the packet,called by the kernel when there is an
 * application wants to transmit a packet.
 */
int vnet_tx(struct sk_buff *skb, struct net_device *dev)
{
    int len;
    char *data;
    struct vnet_priv *priv = (struct vnet_priv *)netdev_priv(dev);

    if (vd[VD_TX_DEVICE].busy ==1) {
        return -EBUSY;
    }

    len = skb->len < ETH_ZLEN ? ETH_ZLEN : skb->len;
    data = skb->data;
    /* stamp the time stamp */
    dev->trans_start = jiffies;

    /* remember the skb and free it in vnet_hw_tx */
    priv->skb = skb;

    /* pseudo transmit the packet,hehe */
    vnet_hw_tx(data, len, dev);

    return 0;
}

/*
 * Deal with a transmit timeout.
 */
void vnet_tx_timeout (struct net_device *dev)
{
    struct vnet_priv *priv = (struct vnet_priv *)netdev_priv(dev);

    priv->stats.tx_errors++;
    netif_wake_queue(dev);

    return;
}

/*
 * When we need some ioctls.
 */
int vnet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
    return 0;
}

/*
 * ifconfig to get the packet transmitting status.
 */
struct net_device_stats *vnet_stats(struct net_device *dev)
{
    struct vnet_priv *priv = (struct vnet_priv *)netdev_priv(dev);
    return &priv->stats;
}

/*
 * TCP/IP handshake will call this function, if it need.
 */
int vnet_change_mtu(struct net_device *dev, int new_mtu)
{
    int err;
    unsigned long flags;
    struct vnet_priv *priv = (struct vnet_priv *)netdev_priv(dev);
    spinlock_t *lock = &priv->lock;

    /* en, the mtu CANNOT LESS THEN 68 OR MORE THEN 1500. */
    if (new_mtu < 68)
        return -EINVAL;


    spin_lock_irqsave(lock, flags);
    dev->mtu = new_mtu;
    /* realloc the new buffer */

    err = vd_realloc(new_mtu);
    spin_unlock_irqrestore(lock, flags);

    return err;
}

int vnet_header(struct sk_buff *skb,
                 struct net_device *dev,
                 unsigned short type,
                 void *daddr,
                 void *saddr,
                 unsigned int len)
{
    struct ethhdr *eth = (struct ethhdr *)skb_push(skb,ETH_HLEN);

    eth->h_proto = htons(type);
    memcpy(eth->h_source,saddr? saddr : dev->dev_addr,dev->addr_len);
    memcpy(eth->h_dest,   daddr? daddr : dev->dev_addr, dev->addr_len);

    return (dev->hard_header_len);
}

int vnet_rebuild_header(struct sk_buff *skb)
{
    struct ethhdr *eth = (struct ethhdr *)skb_push(skb,ETH_HLEN);
    struct net_device *dev = skb->dev;

    memcpy(eth->h_source, dev->dev_addr ,dev->addr_len);
    memcpy(eth->h_dest,   dev->dev_addr , dev->addr_len);

    return 0;
}

struct file_operations vd_ops = {
    .owner = THIS_MODULE,
    .read = device_read,
    .write = device_write,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 35)
    .compat_ioctl = device_ioctl,
#else
    .ioctl = device_ioctl,
#endif
    .open = device_open,
    .release = device_release,
};

/* initialize the character devices */
int vch_module_init(void)
{
    int err;
    int i;
    int devno;
    struct vd_device *vdev;

    err = vcdev_init();
    if (err != 0) {
        printk("Initialize char device error: %d\n", err);
        goto err_exit;
    }

    for (i = 0; i < 2; i++) {
        vdev = &vd[i];
        devno = MKDEV(vdev->major, i);
        cdev_init(&vdev->vcdev, &vd_ops);
        vdev->vcdev.owner = THIS_MODULE;
        vdev->vcdev.ops = &vd_ops;
        err = cdev_add(&vdev->vcdev, devno, 1);
        if (err != 0) {
            printk("Install the buffer rx device %s fail(%d)\n", VD_RX_DEVICE_NAME, err);
            goto err_exit;
        }
    }

    for (i = 0; i < 2; i++)
        vd[i].buffer_write = serial_buffer_write;

    return 0;

err_exit:
    // TODO: cdev_del??
    return err;
}

/* clean up the character devices */
void vch_module_cleanup(void)
{
    vcdev_release();
}

static void vnet_start(struct net_device *dev)
{
    struct net_device_ops *device_ops = &vnet_device_ops;
    struct vnet_priv *priv = (struct vnet_priv *)netdev_priv(dev);

    ether_setup(dev);

    dev->netdev_ops = device_ops;
    dev->tx_queue_len = 500;
    /* We do not need the ARP protocol. */
    dev->flags |= IFF_NOARP;

    device_ops->ndo_start_xmit = vnet_tx;
    device_ops->ndo_open = vnet_open;
    device_ops->ndo_stop = vnet_stop;
    device_ops->ndo_get_stats = vnet_stats;
    device_ops->ndo_change_mtu = vnet_change_mtu;
    device_ops->ndo_tx_timeout = vnet_tx_timeout;
    device_ops->ndo_do_ioctl = vnet_ioctl;

    memset(priv, 0, sizeof(struct vnet_priv));
    spin_lock_init(&priv->lock);
}

int vnet_module_init(void)
{
    int err;

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 16, 56) // starting 3.17
    vnet = alloc_netdev(sizeof(struct vnet_priv), "veth%d", NET_NAME_PREDICTABLE, vnet_start);
#else
    vnet = alloc_netdev(sizeof(struct vnet_priv), "veth%d", vnet_start);
#endif
    if (vnet == NULL) {
        printk("Unable to allocate device!\n");
        return -ENOMEM;
    }

    if ((err = register_netdev(vnet)))
        printk("vnet: error %i registering virtual network device \"%s\"\n",
                    err, vnet->name);

    //netif_napi_add(vdev, &vv->napi, vnet_rx_poll, napi_budget());

    return err;
}

void vnet_module_cleanup(void)
{
    unregister_netdev(vnet);
    kfree(netdev_priv(vnet));
    free_netdev(vnet);

    return;
}

/* called by the kernel to setup the module*/
int vd_device_init(void)
{
    int err;

    err = vch_module_init();
    if(err < 0)
        return err;

    err = vnet_module_init();
    if(err < 0)
        return err;

    return err;
}

/* cleanup the module */
void vd_device_exit(void)
{
    vch_module_cleanup();
    vnet_module_cleanup();
}

module_init(vd_device_init);
module_exit(vd_device_exit);
