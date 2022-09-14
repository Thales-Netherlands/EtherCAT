/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2008  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT master userspace library.
 *
 *  The IgH EtherCAT master userspace library is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU Lesser General
 *  Public License as published by the Free Software Foundation; version 2.1
 *  of the License.
 *
 *  The IgH EtherCAT master userspace library is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with the IgH EtherCAT master userspace library. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

/** \file
 *
 * EtherCAT virtual TTY userspace interface.
 *
 * \defgroup TTYInterface EtherCAT Virtual TTY Interface
 *
 * @{
 */

/*****************************************************************************/

#ifndef __ECTTY_USER_H__
#define __ECTTY_USER_H__

/******************************************************************************
 * Data types
 *****************************************************************************/

struct tty_char_dev;
typedef struct tty_char_dev tty_char_dev_t;

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * Global functions
 *****************************************************************************/

/** Opens character device to allow communication between kernel and user space
 * \return -1 on error and 0 on succes
 */
int open_ectty_cdev(void);

/** Closes character device
 */
void close_ectty_cdev(void);

/******************************************************************************
 * TTY interface methods
 *****************************************************************************/

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

/** @} */

#endif
