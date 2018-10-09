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

#include "pdo_out_publisher.h"
#include "ighm_ros/PDOOut.h"
#include "ighm_ros/PDORaw.h"
#include "ethercat_slave.h"
#include "utilities.h"
#include "vector"
#include "ighm_ros.h"
#include <iostream>
#include <string>
namespace PDOOutPublisher
{
ros::Publisher pdo_out_pub;
void pdo_raw_callback(const ighm_ros::PDORaw::ConstPtr &pdo_raw)
{
    std::vector<uint8_t> pdo_out_raw = pdo_raw->pdo_out_raw;
    uint8_t *data_ptr;
    size_t pos;
    for (int i = 0; i < master_info.slave_count; i++)
    {
        pos = i * num_process_data_out; //The size of every entry is num_process_data_out
        data_ptr = (uint8_t *)&pdo_out_raw[pos];
        ighm_ros::PDOOut pdo_out;
        pdo_out.slave_id = i;
        using namespace utilities;

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
        pdo_out_pub.publish(pdo_out);
    }
}

void init(ros::NodeHandle &n)
{
    //Create  ROS subscriber for the Ethercat RAW data
    // pdo_raw_sub_ = n.subscribe("pdo_raw", 1000, &PDOOutPublisher::pdo_raw_callback, &pdo_out_publisher);

    //Create  ROS publisher for the Ethercat formatted data
    ros::Publisher pdo_out_pub = n.advertise<ighm_ros::PDOOut>("pdo_out", 1000);
}
} // namespace PDOOutPublisher
