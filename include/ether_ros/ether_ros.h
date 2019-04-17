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
 *  Public License as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace program in the ROS environment. If not, see
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
/**
   \file ether_ros.h
   \brief Main header file.
*/

/*****************************************************************************/
/** \var uint8_t *domain1_pd
    \brief Global buffer for the actual communication with the IgH Master Module.
*/
/** \var uint8_t *process_data_buf
    \brief Global buffer for safe concurrent accesses from the output PDOs services and the EtherCAT Communicator. \see ethercat_comm
*/
/** \var size_t total_process_data
    \brief Total number of process data (PD) (bytes).
*/
/** \var size_t num_process_data_in
    \brief Number of input PD per slave (bytes).

    Assumes that the EtherCAT application is the same for every slave.
*/
/** \var size_t num_process_data_out
    \brief Number of output PD per slave (bytes).

    Assumes that the EtherCAT application is the same for every slave.
*/
/** \var int log_fd
    \brief File descriptor used for logging, provided that TIMING_SAMPLING is enabled.

    Could be deprecated in a next version (see kernelshark).
*/
/** \var ec_master_t *master
    \brief The main master struct.

    Used for communication with the IgH Master Module.
*/
/** \var ec_master_state_t master_state
    \brief The master state struct.

    Used to examine the current state (Links Up/Down, AL states) of the Master.
*/
/** \var ec_master_info_t master_info
    \brief The master info struct.

    Used to know the slaves responding to the Master.
*/
/** \var ec_domain_t *domain1
    \brief The main domain struct variable.

    Used to send and receive the datagrams.
*/
/** \var ec_domain_state_t domain1_state
    \brief The domain state struct.

    Used to examine the current state (Working counter, DL states) of the domain. \see ethercat_comm
*/
/** \var slave_struct *ethercat_slaves
    \brief The main slave struct.

    Used by our program to contain all the useful info of every slave.
*/
/** \var pthread_spinlock_t lock
    \brief The shared spinlock.

    Used by every thread whick modifies the process_data_buf. \see process_data_buf
*/
/** \var EthercatCommunicator ethercat_comm
    \brief The barebone object of our application.

    Used for realtime communication (Tx/Rx) with the EtherCAT slaves.
    Doesn't change the output PDOs. Basic state machine:
    -  Receive the new PDOs in domain1_pd from the IgH Master Module (and then to EtherCAT slaves)
    - Move to the domain_pd the output data of process_data_buf, safely
    - Publish the "raw" data (not linked to EtherCAT variables) in PDOs from the domain1_pd
    - Send the new PDOs from domain1_pd to the IgH Master Module (and then to EtherCAT slaves)
*/
/** \var EthercatInputDataHandler pdo_in_publisher
    \brief Main object for publishing to the /ethercat_data_slave_x the values of the EtherCAT input variables.

    Maps indeces to variables.
*/
/** \var EthercatOutputDataHandler pdo_out_publisher
    \brief Main object for publishing to the /ethercat_data_out the values of the EtherCAT output variables.

    Maps indeces to variables.
*/
/** \var int FREQUENCY /**<
    \brief Frequency of the realtime thread: EtherCAT Communicator.
*/
/** \var int RUN_TIME
    \brief Total run time of the realtime thread: EtherCAT Communicator.
*/
/** \var int PERIOD_NS
    \brief Handy variable induced from the Frequency variable. \see FREQUENCY
*/
/** \def CLOCK_TO_USE CLOCK_MONOTONIC
    \brief A macro that helps to try different clocks easily.

    The CLOCK_MONOTONIC is the best for
    realtime purposes.
*/
/** \def SAMPLING_FREQ 10
    \brief The sampling frequency.

    Used when the \a TIMING_SAMPLING is defined.
    It's used as a frequency for sampling measurements.
    The number is in Hz.
*/
/** \def NSEC_PER_SEC (1000000000L)
    \brief Nanoseconds per second.

    Used to get the \a PERIOD_NS from \a FREQUENCY.
*/
/** \def DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + (B).tv_nsec - (A).tv_nsec)
    \brief A difference in nanoseconds macro.

    Used in the realtime \a while loop.
*/
/** \def handle_error_en(en, msg)
    do
    {
        errno = en;
        ROS_FATAL(msg);
        exit(EXIT_FAILURE);
    } while (0)
    \brief Handle error in the ROS context.
*/
/** \def TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
    \brief Macro that returns the nanoseconds from the timespec struct.

*/
/** \typedef struct slave_struct slave_struct
    \brief A shorthand definition of slave_struct

*/
/** \struct slave_struct
    \brief The basic slave struct.
    \var slave_struct::slave_name
    \brief The slave's name (for convienience, the names are the positions of the legs)
    \var slave_struct::id
    \brief The slave's id (in a four legged robot, that will be from 0 to 3)
    \var slave_struct::slave
    \brief The EthercatSlave object, used to store every other useful information.

*/

#ifndef ether_ros_LIB_H
#define ether_ros_LIB_H

#include "ros/ros.h"
// #include <errno.h>
#include <stdio.h>
// #include <string.h>
// #include <sys/resource.h>
#include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
#include <unistd.h>
// #include <time.h>
#include <sys/mman.h>
#include <stddef.h>
#include "ecrt.h"
#include "ethercat_slave.h"
#include "ethercat_communicator.h"
#include "pdo_in_publisher.h"
#include "pdo_out_publisher.h"
#include "pdo_out_listener.h"
#include "pdo_out_publisher_timer.h"

// Application parameters
#define CLOCK_TO_USE CLOCK_MONOTONIC
// #define TIMING_SAMPLING 0
// #define RUN_TIME 60 // run time in seconds
#if TIMING_SAMPLING
#define SAMPLING_FREQ 10
#endif
/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
// #define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/** Return the sign of a number
 *
 * ie -1 for -ve value, 0 for 0, +1 for +ve value
 *
 * \retval the sign of the value
 */
#define sign(val) \
    ({ typeof (val) _val = (val); \
    ((_val > 0) - (_val < 0)); })

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
extern PDOInPublisher pdo_in_publisher;
extern PDOOutPublisher pdo_out_publisher;
extern PDOOutListener pdo_out_listener;
extern PDOOutPublisherTimer pdo_out_publisher_timer;
extern int PERIOD_NS;
extern int FREQUENCY;
extern int RUN_TIME;
extern statistics_struct stat_struct;
#endif /* ether_ros_LIB_H */
