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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/slab.h>


#include "ioctl.h"
#include "../tty/tty.h"

/*****************************************************************************/

/** Set to 1 to enable ioctl() latency tracing.
 *
 * Requires CPU timestamp counter!
 */
#define DEBUG_LATENCY 0

/** Optional compiler attributes fo ioctl() functions.
 */
#if 0
#define ATTRIBUTES __attribute__ ((__noinline__))
#else
#define ATTRIBUTES
#endif

/*****************************************************************************/

/** Get module information.
 *
 * \return Zero on success, otherwise a negative error code.
 */
static ATTRIBUTES int ec_tty_ioctl_module(
        void *arg /**< Userspace address to retrieve inputs and store the results. */
        )
{
    uint32_t data = EC_TTY_IOCTL_VERSION_MAGIC;

    if (copy_to_user((void __user *) arg, &data, sizeof(data)) != 0)
        return -EFAULT;

    return 0;
}

/*****************************************************************************/

/** Create new tty device
 *
 * \return Zero on success, otherwise a negative error code.
 */
static ATTRIBUTES int ec_tty_ioctl_create(
        void *arg /**< Userspace address to retrieve inputs and store the results. */
        )
{
	ec_tty_t *tty;
	ec_ioctl_create_tty_t data;

    if (copy_from_user(&data, (void __user *) arg, sizeof(data)) != 0) {
        return -EFAULT;
    }

    tty = ectty_create(data.ops, data.cb_data);

    if (IS_ERR(tty))
    {
        return PTR_ERR(tty);
    }

	data.minor = tty->minor;

    if (copy_to_user((void __user *) arg, &data, sizeof(data)) != 0)
        return -EFAULT;

    return 0;
}

/*****************************************************************************/

/** Free all tty devices
 *
 * \return Zero on success, otherwise a negative error code.
 */
static ATTRIBUTES int ec_tty_ioctl_free(
        void *arg /**< Userspace address to retrieve inputs and store the results. */
        )
{
	int minor;
	ec_tty_t *tty;

    if (copy_from_user(&minor, (void __user *) arg, sizeof(minor)) != 0) {
        return -EFAULT;
    }

    tty = ectty_get_by_minor(minor);

    if (IS_ERR(tty))
    {
        return PTR_ERR(tty);
    }

    ectty_free(tty);
    return 0;

}

/*****************************************************************************/

/** Reads data to send from the TTY interface.
 *
 * \returns amount of copied bytes.
 */
static ATTRIBUTES int ec_tty_ioctl_tx(
        void *arg /**< Userspace address to retrieve inputs and store the results. */
        )
{
	ec_ioctl_tx_t data;
	ec_tty_t *tty;
	uint8_t *buffer;

    if (copy_from_user(&data, (void __user *) arg, sizeof(data)) != 0) {
        return -EFAULT;
    }

    tty = ectty_get_by_minor(data.minor);

    if (IS_ERR(tty)) {
        return PTR_ERR(tty);
    }

    buffer = kmalloc(sizeof(*buffer) * data.size, GFP_KERNEL);
    if (buffer == NULL) {
        return -ENOMEM;
    }

    data.n_copied = ectty_tx_data(tty, buffer, data.size);

    if (copy_to_user((void __user *) data.buffer, buffer, sizeof(*buffer) * data.n_copied) != 0) {
        kfree(buffer);
        return -EFAULT;
    }

    if (copy_to_user((void __user *) arg, &data, sizeof(data)) != 0) {
        kfree(buffer);
        return -EFAULT;
    }

    kfree(buffer);
    return 0;
}

/*****************************************************************************/

/** Pushes received data to the TTY interface.
 *
 */
static ATTRIBUTES int ec_tty_ioctl_rx(
        void *arg /**< Userspace address to retrieve inputs and store the results. */
        )
{
	ec_ioctl_rx_t data;
	ec_tty_t *tty;
	uint8_t *buffer;

    if (copy_from_user(&data, (void __user *) arg, sizeof(data)) != 0) {
        return -EFAULT;
    }

    buffer = kmalloc(sizeof(*buffer) * data.size, GFP_KERNEL);
    if (buffer == NULL) {
        return -ENOMEM;
    }

    if (copy_from_user(buffer, (void __user *) data.buffer, sizeof(*buffer) * data.size) != 0) {
        kfree(buffer);
        return -EFAULT;
    }

    tty = ectty_get_by_minor(data.minor);

    if (IS_ERR(tty)) {
        kfree(buffer);
        return PTR_ERR(tty);
    }

    ectty_rx_data(tty, buffer, data.size);

    if (copy_to_user((void __user *) arg, &data, sizeof(data)) != 0) {
        kfree(buffer);
        return -EFAULT;
    }

    kfree(buffer);
    return 0;
}

/**************************************************************************** */


/** ioctl() function to use.
 */
#ifdef EC_IOCTL_RTDM
#define EC_IOCTL ec_ioctl_rtdm
#else
#define EC_IOCTL ec_ioctl
#endif

/**************************************************************************** */

/** Called when an ioctl() command is issued.
 *
 * \return ioctl() return code.
 */
long EC_IOCTL(
		ec_ioctl_context_t *ctx, /**< Device context. */
        unsigned int cmd, /**< ioctl() command identifier. */
        void *arg /**< ioctl() argument. */
        )
{
#if DEBUG_LATENCY
    cycles_t a = get_cycles(), b;
    unsigned int t;
#endif

    int ret;

    switch (cmd) {
		case EC_TTY_IOCTL_MODULE:
			ret = ec_tty_ioctl_module(arg);
			break;

        case EC_TTY_IOCTL_CREATE:
			if (ctx->writable == 0) {
				ret = -EPERM;
				break;
			}
            ret = ec_tty_ioctl_create(arg);
            break;

        case EC_TTY_IOCTL_FREE:
			if (ctx->writable == 0) {
				ret = -EPERM;
				break;
			}
        	ret = ec_tty_ioctl_free(arg);
            break;
            
        case EC_TTY_IOCTL_TX:
        	ret = ec_tty_ioctl_tx(arg);
            break;

        case EC_TTY_IOCTL_RX:
        	ret = ec_tty_ioctl_rx(arg);
            break;

        default:
            ret = -ENOTTY;
            break;
    }

#if DEBUG_LATENCY
    b = get_cycles();
    t = (unsigned int) ((b - a) * 1000LL) / cpu_khz;
    if (t > 50) {
		printk(KERN_ERR INFO PFX "ioctl(0x%02x) took %u us.\n",
                _IOC_NR(cmd), t);
    }
#endif

    return ret;
}

/*****************************************************************************/
