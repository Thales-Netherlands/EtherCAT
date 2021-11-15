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

#include <linux/version.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/kthread.h>
#include <linux/sched.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
#include <linux/slab.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif

#include "../../include/ecrt.h" // EtherCAT realtime interface

/*****************************************************************************/

// Module parameters
#define FREQUENCY 100
#define PFX "ec_mini: "

static ec_master_t *master = NULL;
static ec_master_state_t master_state = { };
struct semaphore master_sem;

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = { };

static struct timer_list timer;

static uint8_t *domain1_pd = NULL;
static unsigned int off_dig_out;
static unsigned int off_dig_out_bit;
static unsigned int counter = 0;
static unsigned int blink = 0;
const static ec_pdo_entry_reg_t domain1_regs[] = { { 1, 1, 0x00000002, 0x07d83052, 0x7000, 1, &off_dig_out,
        &off_dig_out_bit }, { } };
/* Master 0, Slave 1, "EL2008"
 * Vendor ID:       0x00000002
 * Product code:    0x07d83052
 * Revision number: 0x00110000
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = { { 0x7000, 0x01, 1 }, /* Output */
{ 0x7010, 0x01, 1 }, /* Output */
{ 0x7020, 0x01, 1 }, /* Output */
{ 0x7030, 0x01, 1 }, /* Output */
{ 0x7040, 0x01, 1 }, /* Output */
{ 0x7050, 0x01, 1 }, /* Output */
{ 0x7060, 0x01, 1 }, /* Output */
{ 0x7070, 0x01, 1 }, /* Output */
};

ec_pdo_info_t slave_1_pdos[] = { { 0x1600, 1, slave_1_pdo_entries + 0 }, /* Channel 1 */
{ 0x1601, 1, slave_1_pdo_entries + 1 }, /* Channel 2 */
{ 0x1602, 1, slave_1_pdo_entries + 2 }, /* Channel 3 */
{ 0x1603, 1, slave_1_pdo_entries + 3 }, /* Channel 4 */
{ 0x1604, 1, slave_1_pdo_entries + 4 }, /* Channel 5 */
{ 0x1605, 1, slave_1_pdo_entries + 5 }, /* Channel 6 */
{ 0x1606, 1, slave_1_pdo_entries + 6 }, /* Channel 7 */
{ 0x1607, 1, slave_1_pdo_entries + 7 }, /* Channel 8 */
};

ec_sync_info_t slave_1_syncs[] = { { 0, EC_DIR_OUTPUT, 8, slave_1_pdos + 0, EC_WD_ENABLE }, { 0xff } };

/* Master 0, Slave 2, "EL6601"
 * Vendor ID:       0x00000002
 * Product code:    0x19c93052
 * Revision number: 0x00150000
 */

ec_sync_info_t slave_2_syncs[] = { { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE }, { 1, EC_DIR_INPUT, 0, NULL,
        EC_WD_DISABLE }, { 2, EC_DIR_OUTPUT, 0,
NULL, EC_WD_DISABLE }, { 3, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE }, { 0xff } };

/*****************************************************************************/

//void check_domain1_state(void)
//{
//    ec_domain_state_t ds;
//
//    down(&master_sem);
//    ecrt_domain_state(domain1, &ds);
//    up(&master_sem);
//
//    if (ds.working_counter != domain1_state.working_counter)
//        printk(KERN_INFO PFX "Domain1: WC %u.\n", ds.working_counter);
//    if (ds.wc_state != domain1_state.wc_state)
//        printk(KERN_INFO PFX "Domain1: State %u.\n", ds.wc_state);
//
//    domain1_state = ds;
//}
//
///*****************************************************************************/
//
//void check_master_state(void)
//{
//    ec_master_state_t ms;
//
//    down(&master_sem);
//    ecrt_master_state(master, &ms);
//    up(&master_sem);
//
//    if (ms.slaves_responding != master_state.slaves_responding)
//        printk(KERN_INFO PFX "%u slave(s).\n", ms.slaves_responding);
//    if (ms.al_states != master_state.al_states)
//        printk(KERN_INFO PFX "AL states: 0x%02X.\n", ms.al_states);
//    if (ms.link_up != master_state.link_up)
//        printk(KERN_INFO PFX "Link is %s.\n", ms.link_up ? "up" : "down");
//
//    master_state = ms;
//}
/*****************************************************************************/

int cyclic_task(void *ignore)

{

    // receive process data
//    if (down_trylock(&master_sem)) {
//        printk(KERN_INFO PFX "Cyclic task fail.\n");
//        goto out_cyclic;
//    }
    while (!kthread_should_stop()) {
        printk(KERN_INFO PFX "Cyclic Task Start.\n");

        down(&master_sem);

        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        //   up(&master_sem);

        // check process data state (optional)
        //  check_domain1_state();

        if (counter) {
            counter--;
        } else { // do this at 1 Hz
            counter = FREQUENCY;

            // calculate new process data
            blink = !blink;

            // check for master state (optional)
            //    check_master_state();

        }

        // write process data
        EC_WRITE_U8(domain1_pd + off_dig_out, blink ? 0x06 : 0x09);

        // send process data
        //  down(&master_sem);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);

        up(&master_sem);
        printk(KERN_INFO PFX "Cyclic Task End.\n");

        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(10);
    }
    printk(KERN_INFO PFX "Cyclic Task End.\n");
    return 0;
}

/*****************************************************************************/

void send_callback(void *cb_data) {
    ec_master_t *m = (ec_master_t *) cb_data;

//    if (down_trylock(&master_sem)) {
//            printk(KERN_INFO PFX "Send callback fail.\n");
//            return;
//        }

    down(&master_sem);
    printk(KERN_INFO PFX "Send callback start.\n");

    ecrt_master_send_ext(m);
    printk(KERN_INFO PFX "Send callback end.\n");

    up(&master_sem);

}

/*****************************************************************************/

void receive_callback(void *cb_data) {
    ec_master_t *m = (ec_master_t *) cb_data;

//    if (down_trylock(&master_sem)) {
//        printk(KERN_INFO PFX "Receive callback fail.\n");
//        return;
//    }

    down(&master_sem);

    printk(KERN_INFO PFX "Receive callback start.\n");

    ecrt_master_receive(m);
    printk(KERN_INFO PFX "Receive callback end.\n");

    up(&master_sem);
}

/*****************************************************************************/
struct task_struct *cyclic_task_thread;

int __init init_mini_module(void)
{
    int ret = -1;
    ec_slave_config_t *sc;

    printk(KERN_INFO PFX "Starting...\n");

    master = ecrt_request_master(0);
    if (!master) {
        ret = -EBUSY;
        printk(KERN_ERR PFX "Requesting master 0 failed.\n");
        goto out_return;
    }

    sema_init(&master_sem, 1);
    ecrt_master_callbacks(master, send_callback, receive_callback, master);

    printk(KERN_INFO PFX "Registering domain...\n");
    if (!(domain1 = ecrt_master_create_domain(master))) {
        printk(KERN_ERR PFX "Domain creation failed!\n");
        goto out_release_master;
    }

    if (!(sc = ecrt_master_slave_config(master, 1, 1, 0x00000002, 0x07d83052))) {
        printk(KERN_ERR PFX "Failed to get slave configuration.\n");
        goto out_release_master;
    }

    if (ecrt_slave_config_pdos(sc, EC_END, slave_1_syncs)) {
        printk(KERN_ERR PFX "Failed to configure PDOs.\n");
        goto out_release_master;
    }

    printk(KERN_INFO PFX "Registering PDO entries...\n");
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        printk(KERN_ERR PFX "PDO entry registration failed!\n");
        goto out_release_master;
    }

    printk(KERN_INFO PFX "Activating master...\n");
    if (ecrt_master_activate(master)) {
        printk(KERN_ERR PFX "Failed to activate master!\n");

        goto out_release_master;

    }
    // Get internal process data for domain
    domain1_pd = ecrt_domain_data(domain1);

    printk(KERN_INFO PFX "Starting cyclic sample thread.\n");

    cyclic_task_thread = kthread_run(cyclic_task, NULL, "CYCLIC_TASK");
    if (IS_ERR(cyclic_task_thread)) {
        printk(KERN_INFO PFX "Cyclic Task Thread failed.\n");
        goto out_release_master;
    }
    printk(KERN_INFO PFX "Started.\n");
    return 0;

    out_release_master:
    printk(KERN_ERR PFX "Releasing master...\n");
    ecrt_release_master(master);
    out_return:
    printk(KERN_ERR PFX "Failed to load. Aborting.\n");
    return ret;
}

/*****************************************************************************/

void __exit cleanup_mini_module(void)
{
    printk(KERN_INFO PFX "Stopping...\n");

    kthread_stop(cyclic_task_thread);
    printk(KERN_INFO PFX "Releasing master...\n");
    ecrt_release_master(master);

    printk(KERN_INFO PFX "Unloading.\n");
}

/*****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florian Pose <fp@igh-essen.com>");
MODULE_DESCRIPTION("EtherCAT minimal test environment");

module_init( init_mini_module);
module_exit( cleanup_mini_module);

/*****************************************************************************/
