/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2018 Mike Karamousadakis, NTUA CSL
 *
 *  This file is part of the IgH EtherCAT master userspace program in the ROS environment.
 *
 *  The IgH EtherCAT master userspace program in the ROS environment is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation; version 3
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace library. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: mkaramousadakis@zoho.eu
 *****************************************************************************/

#ifndef IGHM_ROS_LIB_H
#define IGHM_ROS_LIB_H

#include "ros/ros.h"
#include <errno.h>
// #include <stdio.h>
// #include <string.h>
// #include <sys/resource.h>
#include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
#include <unistd.h>
// #include <time.h>
#include <sys/mman.h>
#include <stddef.h>
// #include "ecrt.h"
// #include "ethercat_slave.h"
// #include "ethercat_communicator.h"
// #include "ethercat_input_data_handler.h"
// #include "ethercat_output_data_handler.h"

// Application parameters
#define CLOCK_TO_USE CLOCK_MONOTONIC
// #define MEASURE_TIMING 2
// #define RUN_TIME 60 // run time in seconds
#if MEASURE_TIMING == 1
#define SAMPLING_FREQ 10
#endif
/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
// #define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

#define handle_error_en(en, msg) \
    do                           \
    {                            \
        errno = en;              \
        ROS_FATAL(msg);          \
        exit(EXIT_FAILURE);      \
    } while (0)

/****************************************************************************/

typedef struct slave_struct
{
    std::string slave_name;
    int id;
    EthercatSlave slave;
} slave_struct;

extern slave_struct * ethercat_slaves;
extern uint8_t *domain1_pd;
extern uint8_t * process_data_buf;
extern size_t total_process_data;
extern size_t num_process_data_in;
extern size_t num_process_data_out;
extern int log_fd;
extern ec_master_t *master;
extern ec_master_state_t master_state;
extern ec_master_info_t master_info;
extern ec_domain_t *domain1;
extern ec_domain_state_t domain1_state;
extern pthread_spinlock_t lock;
extern EthercatCommunicator ethercat_comm;
extern EthercatInputDataHandler ethercat_input_data_handler;
extern EthercatOutputDataHandler ethercat_output_data_handler;
extern int PERIOD_NS;
extern int FREQUENCY;
extern int RUN_TIME;
#endif /* IGHM_ROS_LIB_H */
