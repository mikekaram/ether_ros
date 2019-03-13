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
   \file pdo_in_publisher.cpp
   \brief Implementation of PDOInPublisher class.

   Used for publishing the "raw" input data, received from EtherCAT Communicator after transformation into
   useful, human-readable format, consisted of the EtherCAT variables used by our
   application. Transforms the indeces to variables.
*/

/*****************************************************************************/
#include "pdo_in_publisher.h"
#include "ether_ros/PDOIn.h"
#include "ether_ros/PDORaw.h"
#include "ethercat_slave.h"
#include "utilities.h"
#include "vector"
#include "ether_ros.h"
#include <iostream>
#include <string>

void PDOInPublisher::pdo_raw_callback(const ether_ros::PDORaw::ConstPtr &pdo_raw)
{
    std::vector<uint8_t> pdo_in_raw = pdo_raw->pdo_in_raw;
    uint8_t *data_ptr;
    size_t pos;
    for (int i = 0; i < master_info.slave_count; i++)
    {
        pos = i * num_process_data_in; //The size of every entry is num_process_data_in
        data_ptr = (uint8_t * ) & pdo_in_raw[pos];
        ether_ros::PDOIn pdo_in;
        using namespace utilities;

        // change the following code to match your needs
        /*

        Insert code here ...

        */

        pdo_in.hip_angle = process_input_int16(data_ptr, 0);
        pdo_in.desired_hip_angle = process_input_int16(data_ptr, 2);
        pdo_in.time = process_input_uint16(data_ptr, 4);
        pdo_in.knee_angle = process_input_int16(data_ptr, 6);
        pdo_in.desired_knee_angle = process_input_int16(data_ptr, 8);
        pdo_in.PWM10000_knee = process_input_int16(data_ptr, 10);
        pdo_in.PWM10000_hip = process_input_int16(data_ptr, 12);
        pdo_in.velocity_knee1000 = process_input_int32(data_ptr, 14);
        pdo_in.velocity_hip1000 = process_input_int32(data_ptr, 18);

        /*
            .....

        */
        pdo_in_pub_[i].publish(pdo_in);
    }
}

void PDOInPublisher::init(ros::NodeHandle &n)
{
    //Create  ROS subscriber for the Ethercat RAW data
    pdo_raw_sub_ = n.subscribe("pdo_raw", 1000, &PDOInPublisher::pdo_raw_callback, &pdo_in_publisher);

    //Create  ROS publishers for the Ethercat formatted data
    pdo_in_pub_ = new ros::Publisher[master_info.slave_count];
    for (int i = 0; i < master_info.slave_count; i++)
    {
        pdo_in_pub_[i] = n.advertise<ether_ros::PDOIn>("pdo_in_slave_" + std::to_string(i), 1000);
    }
}
