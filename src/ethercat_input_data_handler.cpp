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

#include "ethercat_input_data_handler.h"
#include "ighm_ros/EthercatInputData.h"
#include "ighm_ros/EthercatRawData.h"
#include "ethercat_slave.h"
#include "utilities.h"
#include "vector"
#include "ighm_ros.h"
#include <iostream>
#include <string>

void EthercatInputDataHandler::raw_data_callback(const ighm_ros::EthercatRawData::ConstPtr &ethercat_data_raw)
{
    std::vector<uint8_t> input_data_raw = ethercat_data_raw->input_data_raw;
    uint8_t *data_ptr;
    size_t pos;
    for (int i = 0; i < master_info.slave_count; i++)
    {
        pos = i * num_process_data_in; //The size of every entry is num_process_data_in
        data_ptr = (uint8_t * ) & input_data_raw[pos];
        ighm_ros::EthercatInputData ethercat_input_data;
        ethercat_input_data.hip_angle = process_input_sint16(data_ptr, 0);
        ethercat_input_data.desired_hip_angle = process_input_sint16(data_ptr, 2);
        ethercat_input_data.time = process_input_uint16(data_ptr, 4);
        ethercat_input_data.knee_angle = process_input_sint16(data_ptr, 6);
        ethercat_input_data.desired_knee_angle = process_input_sint16(data_ptr, 8);
        ethercat_input_data.PWM10000_knee = process_input_sint16(data_ptr, 10);
        ethercat_input_data.PWM10000_hip = process_input_sint16(data_ptr, 12);
        ethercat_input_data.velocity_knee1000 = process_input_sint32(data_ptr, 14);
        ethercat_input_data.velocity_hip1000 = process_input_sint32(data_ptr, 18);
        data_pub_[i].publish(ethercat_input_data);
    }
}

void EthercatInputDataHandler::init(ros::NodeHandle &n)
{
    //Create  ROS subscriber for the Ethercat RAW data
    data_raw_sub_ = n.subscribe("ethercat_data_raw", 1000, &EthercatInputDataHandler::raw_data_callback, &ethercat_input_data_handler);

    //Create  ROS publishers for the Ethercat formatted data
    data_pub_ = new ros::Publisher[master_info.slave_count];
    for (int i = 0; i < master_info.slave_count; i++)
    {
        data_pub_[i] = n.advertise<ighm_ros::EthercatInputData>("ethercat_data_slave_" + std::to_string(i), 1000);
    }
}
