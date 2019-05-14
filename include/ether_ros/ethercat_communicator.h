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
   \file ethercat_communicator.h
   \brief Header file for the EthercatCommunicator class
*/

/*****************************************************************************/
#ifndef ETH_COM_LIB_H
#define ETH_COM_LIB_H

#include <iostream>
#include <pthread.h>
#include "ros/ros.h"
#include <sched.h>
/** \class EthercatCommunicator
    \brief The Ethercat Communicator class.

    Basic class for implementing realtime pure communication purposes,
    from our application to the Ethercat slaves, via IgH Master module.
    The class uses the POSIX API for gaining realtime attributes.
*/
#define DC_FILTER_CNT 1024
#if !defined(SYNC_MASTER_TO_REF) && !defined(SYNC_REF_TO_MASTER)

#define SYNC_REF_TO_MASTER //the default synchronization will be ref to master

#endif
#if !defined(FIFO_SCHEDULING) && !defined(DEADLINE_SCHEDULING)

#define FIFO_SCHEDULING //the default scheduling policy will be FIFO

#endif
#ifdef LOGGING
#ifdef LOGGING_SAMPLING

typedef struct statistics_struct
{
  int statistics_id;
  uint32_t period_ns;
  uint32_t exec_ns;
  uint32_t latency_ns;
  uint32_t * latency_min_ns;
  uint32_t * latency_max_ns;
  uint32_t * period_min_ns;
  uint32_t * period_max_ns;
  uint32_t * exec_min_ns;
  uint32_t * exec_max_ns;
  struct timespec start_time;
  struct timespec end_time;
  struct timespec last_start_time;
} statistics_struct;
#endif
#ifdef LOGGING_NO_SAMPLING

typedef struct statistics_struct
{
  int statistics_id;
  uint32_t * period_ns;
  uint32_t * exec_ns;
  uint32_t * latency_ns;
  struct timespec start_time;
  struct timespec end_time;
  struct timespec last_start_time;
} statistics_struct;
#endif
#endif
class EthercatCommunicator
{
private:
  pthread_attr_t current_thattr_;
  struct sched_param sched_param_;
  static int cleanup_pop_arg_;
  //cleanup_pop_arg_ is used only for future references. No actual usage in our application.
  //Serves as an argument to the cleanup_handler.
  static pthread_t communicator_thread_;
  static ros::Publisher pdo_raw_pub_;
  static bool running_thread_;
  static uint64_t dc_start_time_ns_;
  static uint64_t dc_time_ns_;
  static int64_t system_time_base_;

#ifdef SYNC_MASTER_TO_REF
  static uint8_t dc_started_;
  static int32_t dc_diff_ns_;
  static int32_t prev_dc_diff_ns_;
  static int64_t dc_diff_total_ns_;
  static int64_t dc_delta_total_ns_;
  static int dc_filter_idx_;
  static int64_t dc_adjust_ns_;
#endif
  static void *run(void *arg);
  static void cleanup_handler(void *arg);
  static void copy_data_to_domain_buf();
  static void publish_raw_data();
  static void sync_distributed_clocks(void);
  static void update_master_clock(void);
  static uint64_t system_time_ns(void);
#ifdef LOGGING
  static void create_new_statistics_sample(statistics_struct *ss, unsigned int * sampling_counter);
  static void create_statistics(statistics_struct * ss, struct timespec * wakeup_time_p);
  static void log_statistics_to_file(statistics_struct *ss);
#endif
public:
/** \fn static bool has_running_thread()
    \brief A getter for knowing if there is a running thread.

    It's used from the EthercatCommd service, to know if a user stops/starts an already stopped/started EthercatCommunicator.

*/
/** \fn void init(ros::NodeHandle &n)
    \brief Initializes the main thread.

    Mostly makes ready the attributes of the realtime thread, before running.
    \param n The ROS Node Handle

*/
/** \fn void start()
    \brief Starts the main thread.

    The function that actually starts the realtime thread. The realtime attributes have been set from \a init.
    Implements the basic realtime communication (Tx/Rx) with the EtherCAT slaves.
    Doesn't change the output PDOs. Basic state machine:
    -  Receive the new PDOs in domain1_pd from the IgH Master Module (and therefore from the EtherCAT slaves)
    - Move to the domain_pd the output data of process_data_buf, safely
    - Publish the "raw" data (not linked to EtherCAT variables) in PDOs received from the domain1_pd, to the /ethercat_data_raw topic
    - Synchronize the DC of every slave (every \a count'nth cycle)
    - Send the new PDOs from domain1_pd to the IgH Master Module (and then to EtherCAT slaves)
    \see void init(ros::NodeHandle &n)

*/
/** \fn void stop()
    \brief Stops the main thread.

    This function stops the execution of the realtime thread. The mechanism for stopping it,
    is provided by the POSIX API. Search for \a pthread_testcancel() and other related functions.

*/
  static bool has_running_thread();
  void init(ros::NodeHandle &n);
  void start();
  void stop();
};
#endif /* ETH_COM_LIB_H */
