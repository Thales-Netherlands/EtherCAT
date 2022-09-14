#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "tty.h"
#include "ioctl.h"
#include "cdev.h"


/****************************************************************************/

struct tty_char_dev *char_dev;

/****************************************************************************/

int open_ectty_cdev(void) {

	printf("Setting up communication with character device.. \n");
	char_dev = ectty_open_char_device();
	if (char_dev == NULL)
	{
	    return -1;
	}
    return 0;
}

/****************************************************************************/

void close_ectty_cdev(void) {

	if(char_dev != NULL)
	{
		printf("Releasing communication with character device.. \n");
		ectty_release_char_device(char_dev);
		char_dev = NULL;
	}
}

/****************************************************************************/

ec_tty_t *ectty_create(
        const ec_tty_operations_t *ops,
        void *cb_data
        )
{
	ec_ioctl_create_tty_t data;
	ec_tty_t *tty;
    int ret;

    if(char_dev == NULL){
        fprintf(stderr, "IOCTL call on closed character device: %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
        return NULL;
    }

	tty = malloc(sizeof(ec_tty_t));
    if (tty == NULL) {
        fprintf(stderr, "Failed to allocate memory.\n");
        return NULL;
    }

    if (ops != NULL || cb_data != NULL) // should both be NULL
    {
    	fprintf(stderr, "Cflag change callbacks not supported!\n");
        free(tty);
    	return NULL;
    }

    data.ops = ops;
    data.cb_data = cb_data;

    ret = ioctl(char_dev->fd, EC_TTY_IOCTL_CREATE, &data);
    if (EC_IOCTL_IS_ERROR(ret)) {
        fprintf(stderr, "Failed to create tty device: %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
        free(tty);
        return NULL;
    }

    tty->minor = data.minor;

    return tty;

}

/****************************************************************************/

void ectty_free(ec_tty_t *tty)
{
	int minor = tty->minor;
	int ret;

    if(!char_dev){
        fprintf(stderr, "IOCTL call on closed character device: %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
        return;
    }

    ret = ioctl(char_dev->fd, EC_TTY_IOCTL_FREE, &minor);

    if (EC_IOCTL_IS_ERROR(ret)) {
        fprintf(stderr, "Failed to free tty device: %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
    }
    else
    {
        free(tty);
    }

    return;

}

/****************************************************************************/

unsigned int ectty_tx_data(
        ec_tty_t *tty,
        uint8_t *buffer,
        size_t size
        )
{
	ec_ioctl_tx_t data;
	int ret;

    if(!char_dev){
        fprintf(stderr, "IOCTL call on closed character device: %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
        return 0;
    }

	data.minor = tty->minor;
	data.buffer = buffer;
	data.size = size;

    ret = ioctl(char_dev->fd, EC_TTY_IOCTL_TX, &data);

    if (EC_IOCTL_IS_ERROR(ret)) {
        fprintf(stderr, "Failed retreive tx data: %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
        return 0;
    }

    return data.n_copied;
}

/****************************************************************************/

void ectty_rx_data(
        ec_tty_t *tty,
        const uint8_t *buffer,
        size_t size
        )
{
	ec_ioctl_rx_t data;
	int ret;

    if(!char_dev){
        fprintf(stderr, "IOCTL call on closed character device: %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
        return;
    }

	data.minor = tty->minor;
	data.buffer = buffer;
	data.size = size;

    ret = ioctl(char_dev->fd, EC_TTY_IOCTL_RX, &data);

    if (EC_IOCTL_IS_ERROR(ret)) {
        fprintf(stderr, "Failed send rx data: %s\n",
                strerror(EC_IOCTL_ERRNO(ret)));
    }

    return;
}



