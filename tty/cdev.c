/******************************************************************************
 *
 *  Copyright (C) 2006-2020  Florian Pose, Ingenieurgemeinschaft IgH
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

#include <linux/module.h>
#include <linux/version.h>
#include <linux/slab.h>

#include "cdev.h"
#include "ioctl.h"
#include "tty.h"
#include "../master/globals.h"


extern unsigned int debug_level;
extern unsigned int ioctl_debug_filter;

/*****************************************************************************/
extern void ectty_free_all(void);

/*****************************************************************************/
#include <linux/mm.h>
static int ec_cdev_open(struct inode *, struct file *);
static int ec_cdev_release(struct inode *, struct file *);
static long ec_cdev_ioctl(struct file *, unsigned int, unsigned long);

/*****************************************************************************/

/** File operation callbacks for the EtherCAT character device.
 */
static struct file_operations eccdev_fops = {
    .owner          = THIS_MODULE,
    .open           = ec_cdev_open,
    .release        = ec_cdev_release,
    .unlocked_ioctl = ec_cdev_ioctl,
};

/*****************************************************************************/

/** Private data structure for file handles.
 */
typedef struct {
	struct cdev *cdev; /**< Character device. */
    ec_ioctl_context_t ctx; /**< Context. */
} ec_cdev_priv_t;

/*****************************************************************************/

/** Constructor.
 *
 * \return 0 in case of success, else < 0
 */
int ec_cdev_init(
		struct cdev *cdev, /**< EtherCAT master character device. */
        dev_t dev_num /**< Device number. */
        )
{
    int ret;

    cdev_init(cdev, &eccdev_fops);
    cdev->owner = THIS_MODULE;

    ret = cdev_add(cdev,
            MKDEV(MAJOR(dev_num), 0), 1);
    if (ret != 0) {
		printk(KERN_ERR PFX "Failed to add character device!\n");
    }

    return ret;
}

/*****************************************************************************/

/** Destructor.
 */
void ec_cdev_clear(struct cdev *cdev /**< EtherCAT XML device */)
{
    cdev_del(cdev);
}

/******************************************************************************
 * File operations
 *****************************************************************************/

/** Called when the cdev is opened.
 */
int ec_cdev_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
    ec_cdev_priv_t *priv;

    priv = kmalloc(sizeof(ec_cdev_priv_t), GFP_KERNEL);
    if (priv == NULL) {
		printk(KERN_ERR PFX "Failed to allocate memory for private data structure.\n");
        return -ENOMEM;
    }

    priv->cdev = cdev;
    priv->ctx.writable = (filp->f_mode & FMODE_WRITE) != 0;
    priv->ctx.requested = 0;
    priv->ctx.process_data = NULL;
    priv->ctx.process_data_size = 0;

    filp->private_data = priv;

	if (debug_level >= 1)
		printk(KERN_INFO PFX "File opened.\n");

    return 0;
}

/*****************************************************************************/

/** Called when the cdev is closed.
 */
int ec_cdev_release(struct inode *inode, struct file *filp)
{
    ec_cdev_priv_t *priv = (ec_cdev_priv_t *) filp->private_data;

    ectty_free_all();

    if (priv->ctx.process_data != NULL) {
        vfree(priv->ctx.process_data);
    }

	if (debug_level >= 2)
		printk(KERN_INFO PFX "File closed.\n");


    kfree(priv);
    return 0;
}

/*****************************************************************************/

/** Called when an ioctl() command is issued.
 */
long ec_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    ec_cdev_priv_t *priv = (ec_cdev_priv_t *) filp->private_data;

	if (debug_level >= 1) {
	    if (((ioctl_debug_filter >> _IOC_NR(cmd)) % 2 ) == 0) {
	    	printk(KERN_INFO PFX "ioctl(filp = 0x%p, cmd = 0x%08x (0x%02x), arg = 0x%lx)\n",
	                filp, cmd, _IOC_NR(cmd), arg);
	    }
	}

    return ec_ioctl(&priv->ctx, cmd, (void __user *) arg);
}

/*****************************************************************************/

