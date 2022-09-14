/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2008  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

/**
   \file
   EtherCAT tty character device.
*/

/*****************************************************************************/

#ifndef __EC_TTY_IOCTL_H__
#define __EC_TTY_IOCTL_H__

/*****************************************************************************/

#include "../include/ectty.h"
#include "../include/ectty_user.h"

/*****************************************************************************/

/** \cond */

#define EC_TTY_IOCTL_TYPE 0xa5 //0xa4

#define EC_TTY_IO(nr)           _IO(EC_TTY_IOCTL_TYPE, nr)
#define EC_TTY_IOR(nr, type)   _IOR(EC_TTY_IOCTL_TYPE, nr, type)
#define EC_TTY_IOW(nr, type)   _IOW(EC_TTY_IOCTL_TYPE, nr, type)
#define EC_TTY_IOWR(nr, type) _IOWR(EC_TTY_IOCTL_TYPE, nr, type)

/** EtherCAT master ioctl() version magic.
 *
 * Increment this when changing the ioctl interface!
 */
#define EC_TTY_IOCTL_VERSION_MAGIC 1

// Application interface
#define EC_TTY_IOCTL_MODULE                	EC_TTY_IOR(0x00, uint32_t)
#define EC_TTY_IOCTL_CREATE				   EC_TTY_IOWR(0x01, ec_ioctl_create_tty_t)
#define EC_TTY_IOCTL_FREE					EC_TTY_IOW(0x02, int)
#define EC_TTY_IOCTL_TX  				   EC_TTY_IOWR(0x03, ec_ioctl_tx_t)
#define EC_TTY_IOCTL_RX 					EC_TTY_IOW(0x04, ec_ioctl_rx_t)


/*****************************************************************************/

#define EC_IOCTL_STRING_SIZE 64

/*****************************************************************************/

typedef struct {
    // inputs
    const ec_tty_operations_t *ops;
    void *cb_data;

    // outputs
	int minor;

} ec_ioctl_create_tty_t;

typedef struct {
    // inputs
    int minor;
    uint8_t *buffer;
	size_t size;

    // outputs
	unsigned int n_copied;

} ec_ioctl_tx_t;

typedef struct {
    // inputs
    int minor;
    const uint8_t *buffer;
	size_t size;

} ec_ioctl_rx_t;


/*****************************************************************************/


#ifdef  __KERNEL__

/** Context data structure for file handles.
 */
typedef struct {
    unsigned int writable; /**< Device was opened with write permission. */
    unsigned int requested; /**< Master was requested via this file handle. */
    uint8_t *process_data; /**< Total process data area. */
    size_t process_data_size; /**< Size of the \a process_data. */
} ec_ioctl_context_t;

long ec_ioctl(ec_ioctl_context_t *, unsigned int,
        void __user *);

#ifdef EC_RTDM

long ec_ioctl_rtdm(ec_master_t *, ec_ioctl_context_t *, unsigned int,
        void __user *);
int ec_rtdm_mmap(ec_ioctl_context_t *, void **);

#endif

#endif

/*****************************************************************************/

/** \endcond */

#endif
