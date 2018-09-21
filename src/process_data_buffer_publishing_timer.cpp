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
   \file pdo_out_publisher.cpp
   \brief Implementation of PDOOutPublisher class.

   Used for handling the "raw" output data, received from EtherCAT Communicator and transforming them into
   useful, human-readable format, consisted of the EtherCAT variables used by our
   application. Transforms the indeces to variables.
*/

/*****************************************************************************/

#include "process_data_buffer_publishing_timer.h"
#include "ighm_ros/PDOOut.h"
#include "ethercat_slave.h"
#include "utilities.h"
#include "vector"
#include "ighm_ros.h"
#include <iostream>
#include <string>

void ProcessDataBufferPublishingTimer::timer_callback(const ros::TimerEvent &event)
{
    size_t pos;
    uint8_t *data_ptr;
    using namespace utilities;
    copy_process_data_buffer_to_buf(data_ptr_);
    
    for (int i = 0; i < master_info.slave_count; i++)
    {
        pos = i * num_process_data_out; //The size of every entry is num_process_data_out
        data_ptr = (uint8_t *)(data_ptr_ + pos);
        ighm_ros::PDOOut pdo_out;
        pdo_out.slave_id = i;

        // change the following code to match your needs
        /*

        Insert code here ...

        */
        pdo_out.state_machine = process_input_bit(data_ptr, 0, 0);
        pdo_out.initialize_clock = process_input_bit(data_ptr, 0, 1);
        pdo_out.initialize_angles = process_input_bit(data_ptr, 0, 2);
        pdo_out.inverse_kinematics = process_input_bit(data_ptr, 0, 3);
        pdo_out.blue_led = process_input_bit(data_ptr, 0, 4);
        pdo_out.red_led = process_input_bit(data_ptr, 0, 5);
        pdo_out.button_1 = process_input_bit(data_ptr, 0, 6);
        pdo_out.button_2 = process_input_bit(data_ptr, 0, 7);
        pdo_out.sync = process_input_int8(data_ptr, 1);
        pdo_out.desired_x_value = process_input_int32(data_ptr, 2);
        pdo_out.filter_bandwidth = process_input_uint16(data_ptr, 6);
        pdo_out.desired_y_value = process_input_int32(data_ptr, 8);
        pdo_out.kp_100_knee = process_input_int16(data_ptr, 12);
        pdo_out.kd_1000_knee = process_input_int16(data_ptr, 14);
        pdo_out.ki_100_knee = process_input_int16(data_ptr, 16);
        pdo_out.kp_100_hip = process_input_int16(data_ptr, 18);
        pdo_out.kd_1000_hip = process_input_int16(data_ptr, 20);
        pdo_out.ki_100_hip = process_input_int16(data_ptr, 22);
        pdo_out.x_cntr_traj1000 = process_input_int16(data_ptr, 24);
        pdo_out.y_cntr_traj1000 = process_input_int16(data_ptr, 26);
        pdo_out.a_ellipse100 = process_input_int16(data_ptr, 28);
        pdo_out.b_ellipse100 = process_input_int16(data_ptr, 30);
        pdo_out.traj_freq100 = process_input_int16(data_ptr, 32);
        pdo_out.phase_deg = process_input_int16(data_ptr, 34);
        pdo_out.flatness_param100 = process_input_int16(data_ptr, 36);

        /*
            .....

        */
        process_data_buffer_pub_.publish(pdo_out);
    }
}

void ProcessDataBufferPublishingTimer::init(ros::NodeHandle &n)
{
    data_ptr_ = (uint8_t *)malloc(total_process_data * sizeof(uint8_t));
    memset(data_ptr_, 0, total_process_data); // fill the buffer with zeros

    //Create  ROS publisher for the Ethercat formatted data
    process_data_buffer_pub_ = n.advertise<ighm_ros::PDOOut>("pdo_out_timer", 1000);

    if (!process_data_buffer_pub_)
    {
        ROS_FATAL("Unable to start publisher in ProcessDataTimer\n");
        exit(1);
    }
    else
    {
        ROS_INFO("Started ProcessDataTimer publisher\n");
    }
    pdo_out_timer_ = n.createTimer(ros::Duration(5), &ProcessDataBufferPublishingTimer::timer_callback, &process_data_buffer_publishing_timer);
    //Create  ROS timer
    //first parameter is in seconds...
    
    if (!pdo_out_timer_)
    {
        ROS_FATAL("Unable to start ProcessDataTimer\n");
        exit(1);
    }
    else
    {
        ROS_INFO("Started ProcessDataTimer\n");
    }
    // ros::spinOnce();
}
