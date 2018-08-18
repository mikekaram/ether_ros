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

#ifndef ETH_COM_LIB_H
#define ETH_COM_LIB_H

#include <iostream>
#include <pthread.h>
#include "ros/ros.h"
#include <sched.h>
void check_domain1_state(void);
void check_master_state(void);
class EthercatCommunicator
{
private:
  pthread_attr_t current_thattr_;
  struct sched_param sched_param_;
  static int cleanup_pop_arg_;
  static pthread_t communicator_thread_;
  static ros::Publisher data_raw_pub_;
  static bool running_thread_;
  static void *run(void *arg);
  static void cleanup_handler(void *arg);
  static void copy_data_to_domain_buf();
  static void publish_raw_data();

public:
  static bool has_running_thread();
  void init(ros::NodeHandle &n);
  void start();
  void stop();
};
#endif /* ETH_COM_LIB_H */
