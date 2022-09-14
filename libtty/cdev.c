#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include "ioctl.h"
#include "tty.h"
#include "cdev.h"

#define DEVICE_NAME "EtherCAT_tty"

/*****************************************************************************/

#define MAX_PATH_LEN 64

tty_char_dev_t *ectty_open_char_device()
{
    char path[MAX_PATH_LEN];
    tty_char_dev_t *char_dev = NULL;
    uint32_t ioctl_version_magic;
    int ret;

    char_dev = malloc(sizeof(tty_char_dev_t));
    if (char_dev == NULL) {
        fprintf(stderr, "Failed to allocate memory.\n");
        return NULL;
    }

    snprintf(path, MAX_PATH_LEN - 1,
#ifdef USE_RTDM
            DEVICE_NAME);
#else
            "/dev/" DEVICE_NAME);
#endif

#ifdef USE_RTDM
    char_dev->fd = rt_dev_open(path, O_RDWR);
#else
    char_dev->fd = open(path, O_RDWR);
#endif

    if (EC_IOCTL_IS_ERROR(char_dev->fd)) {
        fprintf(stderr, "Failed to open %s: %s \n", path,
        		strerror(EC_IOCTL_ERRNO(char_dev->fd)));
        goto out_clear;
    }

    ret = ioctl(char_dev->fd, EC_TTY_IOCTL_MODULE, &ioctl_version_magic);
    if (EC_IOCTL_IS_ERROR(ret)) {
        fprintf(stderr, "Failed to get module information from %s: %s\n",
                path, strerror(EC_IOCTL_ERRNO(ret)));
        goto out_clear;
    }

    if (ioctl_version_magic != EC_TTY_IOCTL_VERSION_MAGIC) {
        fprintf(stderr, "ioctl() version magic is differing:"
                " %s: %u, libethercat: %u.\n",
                path, ioctl_version_magic,
				EC_TTY_IOCTL_VERSION_MAGIC);
        goto out_clear;
    }

    return char_dev;

out_clear:
	ectty_char_device_clear(char_dev);
    free(char_dev);
    return NULL;
}

/*****************************************************************************/

void ectty_release_char_device(tty_char_dev_t *char_dev)
{
	ectty_char_device_clear(char_dev);
    free(char_dev);
}

/*****************************************************************************/

void ectty_char_device_clear(tty_char_dev_t *char_dev)
{

    if (char_dev->fd != -1) {
#if USE_RTDM
        rt_dev_close(char_dev->fd);
#else
        close(char_dev->fd);
#endif
        char_dev->fd = -1;
    }
}

/****************************************************************************/






