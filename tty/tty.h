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

#ifndef __EC_TTY_TTY_H__
#define __EC_TTY_TTY_H__

#include <linux/module.h>
#include <linux/version.h>
#include <linux/tty.h>

#include "../include/ectty.h"


#define EC_TTY_MAX_DEVICES 32
#define EC_TTY_TX_BUFFER_SIZE 100
#define EC_TTY_RX_BUFFER_SIZE 100

#define PFX "EtherCAT_tty: "
#define DEVICE_NAME "EtherCAT_tty"

/*****************************************************************************/

static const struct tty_operations ec_tty_ops; // see below

/*****************************************************************************/

struct ec_tty {
    int minor;
    struct device *dev;

    uint8_t tx_buffer[EC_TTY_TX_BUFFER_SIZE];
    unsigned int tx_read_idx;
    unsigned int tx_write_idx;
    unsigned int wakeup;

    uint8_t rx_buffer[EC_TTY_RX_BUFFER_SIZE];
    unsigned int rx_read_idx;
    unsigned int rx_write_idx;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
    struct tty_port port;
#endif

    struct timer_list timer;
    struct tty_struct *tty;
    unsigned int open_count;
    struct semaphore sem;

    ec_tty_operations_t ops;
    void *cb_data;
};

/*****************************************************************************/

ec_tty_t *ectty_get_by_minor(int minor);

/*****************************************************************************/

#endif
