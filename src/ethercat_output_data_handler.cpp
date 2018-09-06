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
   \file ethercat_output_data_handler.cpp
   \brief Implementation of EthercatOutputDataHandler class.

   Used for handling the "raw" output data, received from EtherCAT Communicator and transforming them into
   useful, human-readable format, consisted of the EtherCAT variables used by our
   application. Transforms the indeces to variables.
*/

/*****************************************************************************/

#include "ethercat_output_data_handler.h"
#include "ighm_ros/EthercatOutputData.h"
#include "ighm_ros/EthercatRawData.h"
#include "ethercat_slave.h"
#include "utilities.h"
#include <vector>
#include "ighm_ros.h"
#include <iostream>
#include <string>

void EthercatOutputDataHandler::raw_data_callback(const ighm_ros::EthercatRawData::ConstPtr &ethercat_data_raw)
{
    std::vector<uint8_t> output_data_raw = ethercat_data_raw->output_data_raw;
    uint8_t *data_ptr;
    size_t pos;
    for (int i = 0; i < master_info.slave_count; i++)
    {
        pos = i * num_process_data_out; //The size of every entry is num_process_data_out
        data_ptr = (uint8_t *)&output_data_raw[pos];
        ighm_ros::EthercatOutputData ethercat_output_data;
        ethercat_output_data.slave_id = i;
        ethercat_output_data.state_machine = process_input_bit(data_ptr, 0, 0);
        ethercat_output_data.initialize_clock = process_input_bit(data_ptr, 0, 1);
        ethercat_output_data.initialize_angles = process_input_bit(data_ptr, 0, 2);
        ethercat_output_data.inverse_kinematics = process_input_bit(data_ptr, 0, 3);
        ethercat_output_data.blue_led = process_input_bit(data_ptr, 0, 4);
        ethercat_output_data.red_led = process_input_bit(data_ptr, 0, 5);
        ethercat_output_data.button_1 = process_input_bit(data_ptr, 0, 6);
        ethercat_output_data.button_2 = process_input_bit(data_ptr, 0, 7);
        ethercat_output_data.sync = process_input_sint8(data_ptr, 1);
        ethercat_output_data.desired_x_value = process_input_sint32(data_ptr, 2);
        ethercat_output_data.filter_bandwidth = process_input_uint16(data_ptr, 6);
        ethercat_output_data.desired_y_value = process_input_sint32(data_ptr, 8);
        ethercat_output_data.kp_100_knee = process_input_sint16(data_ptr, 12);
        ethercat_output_data.kd_1000_knee = process_input_sint16(data_ptr, 14);
        ethercat_output_data.ki_100_knee = process_input_sint16(data_ptr, 16);
        ethercat_output_data.kp_100_hip = process_input_sint16(data_ptr, 18);
        ethercat_output_data.kd_1000_hip = process_input_sint16(data_ptr, 20);
        ethercat_output_data.ki_100_hip = process_input_sint16(data_ptr, 22);
        ethercat_output_data.x_cntr_traj1000 = process_input_sint16(data_ptr, 24);
        ethercat_output_data.y_cntr_traj1000 = process_input_sint16(data_ptr, 26);
        ethercat_output_data.a_ellipse100 = process_input_sint16(data_ptr, 28);
        ethercat_output_data.b_ellipse100 = process_input_sint16(data_ptr, 30);
        ethercat_output_data.traj_freq100 = process_input_sint16(data_ptr, 32);
        ethercat_output_data.phase_deg = process_input_sint16(data_ptr, 34);
        ethercat_output_data.flatness_param100 = process_input_sint16(data_ptr, 36);
        output_data_pub_.publish(ethercat_output_data);
    }
}

void EthercatOutputDataHandler::init(ros::NodeHandle &n)
{
    //Create  ROS subscriber for the Ethercat RAW data
    data_raw_sub_ = n.subscribe("ethercat_data_raw", 1000, &EthercatOutputDataHandler::raw_data_callback, &ethercat_output_data_handler);

    //Create  ROS publishers for the Ethercat formatted data
    output_data_pub_ = n.advertise<ighm_ros::EthercatOutputData>("ethercat_data_out", 1000);
}
