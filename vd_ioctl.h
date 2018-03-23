#ifndef _VD_IOCTL_H
#define _VD_IOCTL_H

#define MAJOR_NUM_RX    200
#define MAJOR_NUM_TX    201
#define IOCTL_SET_BUSY  _IOWR(MAJOR_NUM_TX,1,int)

#endif
