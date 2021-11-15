/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
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
 ****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h> /* sched_setscheduler() */

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

/** Task period in ns. */
#define PERIOD_NS   (1000000)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */

/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

/****************************************************************************/

// process data
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


/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(void)
{
}

/*****************************************************************************/

void cyclic_task()
{
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // check process data state
    check_domain1_state();

    if (counter) {
        counter--;
    } else { // do this at 1 Hz
        counter = FREQUENCY;

        // calculate new process data
        blink = !blink;

        // check for master state (optional)
        check_master_state();

        // check for slave configuration state(s) (optional)
        check_slave_config_states();
    }


    // write process data
    EC_WRITE_U8(domain1_pd + off_dig_out, blink ? 0x06 : 0x09);

    // send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;
    struct timespec wakeup_time;
    int ret = 0;

    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        return -1;
    }

    if (!(sc = ecrt_master_slave_config(master, 1, 1, 0x00000002, 0x07d83052))) {
        return -1;
    }

    if (ecrt_slave_config_pdos(sc, EC_END, slave_1_syncs)) {
        return -1;
    }


    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

    /* Set priority */

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    /* Lock memory */

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    stack_prefault();

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;

    while (1) {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        cyclic_task();

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    return ret;
}

/****************************************************************************/
