#ifndef _VD_DEVICE_H
#define _VD_DEVICE_H

#define VD_RX_DEVICE       0
#define VD_TX_DEVICE       1

/* define the charactor device name */
#define VD_RX_DEVICE_NAME   "vc_rx"
#define VD_TX_DEVICE_NAME   "vc_tx"

#define VD_MTU              192
#define VD_MAGIC            0x999
#define BUFFER_SIZE         2048

#define VD_TIMEOUT 5

/* These are the flags in the statusword */



struct vd_device{
	int magic;
	char name[8];
	int busy;
	unsigned char *buffer;

    wait_queue_head_t rwait;
	int mtu;
	spinlock_t lock;

	int tx_len;
    int rx_len;
    int buffer_size;
    int major;
    int minor;

    struct cdev vcdev;
	struct file *file;
    ssize_t (*buffer_write)(const char *buffer,size_t length,int buffer_size);
};

/* this is the private data struct of vnet */
struct vnet_priv {
    struct net_device_stats stats;
    struct sk_buff *skb;
    spinlock_t lock;
};

#endif
