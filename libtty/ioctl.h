#ifndef __EC_LIB_IOCTL_H__
#define __EC_LIB_IOCTL_H__

#include "../tty/ioctl.h"

/*****************************************************************************/

#ifdef USE_RTDM
#include <rtdm/rtdm.h>
#else
#include <sys/ioctl.h>
#endif

/*****************************************************************************/

#ifdef USE_RTDM

#define ioctl rt_dev_ioctl

/* rt_dev_ioctl() returns negative error code */
#define EC_IOCTL_IS_ERROR(X) ((X) < 0)
#define EC_IOCTL_ERRNO(X) (-(X))

#else

#define ioctl ioctl

/* libc's ioctl() always returns -1 on error and sets errno */
#define EC_IOCTL_IS_ERROR(X) ((X) == -1)
#define EC_IOCTL_ERRNO(X) (errno)

#include <errno.h>

#endif

/*****************************************************************************/

#endif /* IOCTL_H_ */
