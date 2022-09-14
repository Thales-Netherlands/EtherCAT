#ifndef __EC_LIB_CDEV_H__
#define __EC_LIB_CDEV_H__

#include "../include/ectty.h"
#include "../include/ectty_user.h"


/*****************************************************************************/

struct tty_char_dev {
    int fd;
};

/*****************************************************************************/

tty_char_dev_t *ectty_open_char_device();
void ectty_release_char_device(tty_char_dev_t *char_dev);
void ectty_char_device_clear(tty_char_dev_t *char_dev);

/*****************************************************************************/

#endif /* CDEV_LIB_H_ */
