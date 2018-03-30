/* serial communication server
 * This program's task is to read data from /dev/vc_tx
 * and write data into /dev/vc_rx. /dev/vc_tx is for the
 * network driver to store the tx data; /dev/vc_rx is for
 * the network driver to get the data.
 * Attention: we use AT commands to communicate with ESP8266.
 */


#include <stdio.h>
#include <sys/types.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include "vd_ioctl.h"
#include "server.h"

#define BUFFER_SIZE 2048

/* These defination is from SLIP protocol. */
#define END             0300
#define ESC             0333
#define ESC_END         0334
#define ESC_ESC         0335

int open_com(void)
{
    int fd;

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY );
    if (fd == -1) {
        perror("open_com: Unable to open /dev/ttyUSB0 - ");
        return -1;
    }

    fcntl(fd, F_SETFL, 0);

    return fd;
}

/* set up the options for serial port */
int setup_com(int fd)
{
    struct termios options;
    tcgetattr(fd, &options);
    /*
     * Set the baud rates to 115200...
     */
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    /*
     * Enable the receiver and set local mode...
     */

    options.c_cflag |= (CLOCAL | CREAD);

    /*
     * Set global options.
     */
    options.c_cflag |= PARENB;
    options.c_cflag &= ~PARODD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    /* set the input options */

    options.c_iflag &=~(IXON | IXOFF | IXANY);
    options.c_iflag &=~(INLCR | IGNCR | ICRNL);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* set the output options */
    options.c_oflag &= ~OPOST;

    /* set the timeout options */
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 10;

    tcsetattr(fd, TCSANOW, &options);

    return 1;
}

int open_dev(char *file_name)
{
    int fd;
    fd = open(file_name ,O_RDWR);
    if(fd < 0) {
        perror("Open vc_rx device error:");
        return -1;
    }

    return fd;
}

void close_dev(int fd)
{
    close(fd);
}

/*
 * pack the frame and prepare for sending the packets.
 */
int frame_pack(unsigned char *src, unsigned char *des, int len)
{
    unsigned char *ptr = des;
    unsigned char c;

    *ptr++ = END;
    while (len-- > 0) {
        switch(c = *src++) {
            case END:
                *ptr++ = ESC;
                *ptr++ = ESC_END;
                break;
            case ESC:
                *ptr++ = ESC;
                *ptr++ = ESC_ESC;
                break;
            default:
                *ptr++ = c;
                break;
        }
    }
    *ptr++ = END;
    return (ptr - des);
}

/*
 * unpack the frame.
 */
int frame_unpack(unsigned char *src, unsigned char *des)
{
    unsigned char *ptr = des;
    unsigned char c;
    int count = 0;
    int esc_flag = 0;
    if(*src++ != END)
        return 0;

    while( ((c=*src++) != END) && (count < BUFFER_SIZE)) {
        switch(c) {
            case ESC_END:
                if(esc_flag == 1) {
                    *ptr++ = END;
                    esc_flag = 0;
                }
                else
                    *ptr++ = c;
                break;
            case ESC_ESC:
                if(esc_flag ==1) {
                    *ptr++ = ESC;
                    esc_flag = 0;
                }
                break;
            case END:
                esc_flag = 0;
                break;
            case ESC:
                esc_flag = 1;
                break;
            default:
                *ptr++ = c;
                esc_flag = 0;
                break;
        }
    }
    if(count >= BUFFER_SIZE) {
        printf("some error happen in frame unpack().\n");
        return BUFFER_SIZE;
    }
    return (ptr - des);
}

int main(int argc ,char *argv[])
{
    int fd;
    int fd_rx;
    int fd_tx;

    unsigned char buffer_rx[BUFFER_SIZE];
    unsigned char buffer_tx[BUFFER_SIZE];
    unsigned char buffer_tx_pack[BUFFER_SIZE];

    unsigned char buffer[BUFFER_SIZE];
    unsigned char *buffer_ptr;
    unsigned char *rx_ptr;
    unsigned char *tx_ptr;
    int flag;
    int  nbytes;
    pid_t pid;
    int tx_length;
    int rx_length;
    int i;
    int rx_count;
    int tx_count;

    rx_ptr = buffer_rx;
    tx_ptr = buffer_tx;
    buffer_ptr = buffer;

    fd_rx = open_dev("/dev/vc_rx");
    fd_tx = open_dev("/dev/vc_tx");

    if(fd_rx <0 || fd_tx <0) {
        close(fd);
        return 0;
    }

    fd = open_com();
    if(fd < 0)
        return 0;

    setup_com(fd);
    pid = fork();

    if(pid < 0) {
        printf("Can not creat the process\n");
        close(fd);
        close(fd_rx);
        close(fd_tx);
        return 0;
    }
    if(pid == 0) {
        /* read process */
        while (1) {
            /* read data from /dev/ttyS0, and write the data into buffer_rx,wait for the
               kernel to get the data.
               */
            flag = 0;
            rx_count = 0;
            buffer_ptr = buffer;

            while ((nbytes = read(fd, rx_ptr, BUFFER_SIZE-1)) > 0) {
                rx_length = 0;
                for(i = 0; i < nbytes; i++) {
                    rx_count++;
                    rx_length++;
                    if((buffer_rx[i]) == END) {
                        flag = flag +1;
                        if(flag == 2)
                            break;
                    }
                }


                if(rx_count < BUFFER_SIZE) {
                    memcpy(buffer_ptr,rx_ptr,rx_length);

                    buffer_ptr += rx_length;
                    rx_length = 0;
                }
                /* we recieve two ENDs */
                if(flag == 2) {
                    flag = 0;
                    buffer_ptr = buffer;
                    memset(buffer_rx,0,sizeof(buffer_rx));
                    rx_ptr = buffer_rx;
                    rx_count =frame_unpack(buffer_ptr,rx_ptr);

#ifdef _DEBUG
                    printf("\n Service recieve :\n");
                    for(i=0;i<rx_count;i++)
                        printf(" %02x",rx_ptr[i]&0xff);
                    printf("\n");
#endif
                    write(fd_rx,rx_ptr,rx_count);
                }
            }
        }
    } else if(pid >0) {
        int length;
        /* write process */
        for (;;) {
            /* read data from /dev/ttyS0, and write the data into buffer_rx,wait for the
               kernel to get the data.
               */
            tx_ptr = buffer_tx;
            ioctl(fd_tx,IOCTL_SET_BUSY,1);

            if((nbytes = read(fd_tx, tx_ptr, BUFFER_SIZE-1)) > 0) {
                tx_count = frame_pack(tx_ptr,buffer_tx_pack,nbytes);
#ifdef _DEBUG
                printf("\nSend bytes:%d\n",nbytes);
                printf("\nServer send\n");
                for(i=0;i<tx_count;i++)
                    printf(" %02x",buffer_tx_pack[i]);
                printf("\n");
#endif
                tx_length = 0;
                tx_ptr = buffer_tx_pack;
                while(tx_length < tx_count){
                    length = write(fd,tx_ptr,tx_count-tx_length);
                    tx_length = tx_length + length;
                    tx_ptr += tx_length;
                }
            }

            ioctl(fd_tx,IOCTL_SET_BUSY,0);
        }
    }

    close(fd);
    close(fd_rx);
    close(fd_tx);

    return (-1);
}
