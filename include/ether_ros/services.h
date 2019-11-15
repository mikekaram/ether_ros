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
   \file services.h
   \brief Services header file.

   Includes:
   - EtherCAT Communicator Daemon
   - Services for modifying output PDOs
*/

/*****************************************************************************/

#ifndef SERV_LIB_H
#define SERV_LIB_H

#include "ros/ros.h"
#include "ether_ros/EthercatCommd.h"
/** \fn bool modify_output_bit(ether_ros::ModifyOutputBit::Request &req,
                       ether_ros::ModifyOutputBit::Response &res)
    \brief ROS Service Callback.

    Sets the value of a bit in an index of the \a process_data_buf
*/
/** \fn bool modify_output_sbyte(ether_ros::ModifyOutputBit::Request &req,
                       ether_ros::ModifyOutputBit::Response &res)
    \brief ROS Service Callback.

    Sets the value of a signed byte in an index of the \a process_data_buf
*/
/** \fn bool modify_output_uint16(ether_ros::ModifyOutputUInt16::Request &req,
                          ether_ros::ModifyOutputUInt16::Response &res)
    \brief ROS Service Callback.

    Sets the value of an unsigned 16-bit integer in an index of the \a process_data_buf
*/
/** \fn bool modify_output_sint16(ether_ros::ModifyOutputSInt16::Request &req,
                          ether_ros::ModifyOutputSInt16::Response &res)
    \brief ROS Service Callback.

    Sets the value of a signed 16-bit integer in an index of the \a process_data_buf
*/
/** \fn bool modify_output_sint32(ether_ros::ModifyOutputSInt32::Request &req,
                          ether_ros::ModifyOutputSInt32::Response &res)
    \brief ROS Service Callback.

    Sets the value of a signed 32-bit integer in an index of the \a process_data_buf
*/
/** \fn ethercat_communicatord(ether_ros::EthercatCommd::Request &req,
                            ether_ros::EthercatCommd::Response &res)
    \brief ROS Service Callback.

    Controls the Ethercat Communicator. The basic functionality is:
    - Start
    - Stop
    - Restart
    (Remember that a Service Callback must always return a boolean.)
*/
/** \fn start_ethercat_communicator()
    \brief Helper function for the \a ethercat_communicatord callback.

    Used from the callback in order to actualy send the start command
    to the Ethercat Communicator.
*/
/** \fn stop_ethercat_communicator()
    \brief Helper function for the \a ethercat_communicatord callback.

    Used from the callback in order to actualy send the stop command
    to the Ethercat Communicator.
*/

bool ethercat_communicatord(ether_ros::EthercatCommd::Request &req,
                            ether_ros::EthercatCommd::Response &res);

bool start_ethercat_communicator();
bool stop_ethercat_communicator();

#endif /* SERV_LIB_H */
