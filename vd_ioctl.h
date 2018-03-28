#ifndef _VD_IOCTL_H
#define _VD_IOCTL_H

#define MAJOR_NUM       2
#define MINOR_NUM_RX    0
#define MINOR_NUM_TX    0

#define IOCTL_SET_BUSY  _IOWR(MAJOR_NUM, 1, int)

#endif
