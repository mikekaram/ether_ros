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

#include "modify_pdo_variables_listener.h"
#include "ighm_ros/ModifyPDOVariables.h"
// #include "ethercat_slave.h"
#include "utilities.h"
#include "vector"
#include <pthread.h>
#include "ighm_ros.h"
#include <iostream>
#include <string>

// bool modify_output_sint32(ighm_ros::ModifyOutputSInt32::Request &req,
//                           ighm_ros::ModifyOutputSInt32::Response &res)
// {
//     uint8_t *data_ptr = process_data_buf;
//     int32_t *new_data_ptr = (int32_t *)(data_ptr + req.slave * (num_process_data_out + num_process_data_in) + req.index);
//     pthread_spin_lock(&lock);
//     EC_WRITE_S32(new_data_ptr, req.value);
//     pthread_spin_unlock(&lock);
//     res.success = "true";
//     return true;
// }
void ModifyPDOVariablesListener::modify_pdo_variables_callback(const ighm_ros::ModifyPDOVariables::ConstPtr &new_var)
{
    pthread_spin_lock(&lock);
    uint8_t slave_id = new_var->slave_id;
    //check if we are broadcasting a variable's value to all slaves
    if (slave_id == 255)
    {
        ROS_INFO("slave_id is 255\n");
        for (int i = 0; i < master_info.slave_count; i++)
        {
            modify_pdo_variable(i, new_var);
        }
    }
    else
    {
        modify_pdo_variable((int)slave_id, new_var);
    }
    pthread_spin_unlock(&lock);
}
void ModifyPDOVariablesListener::modify_pdo_variable(int slave_id, const ighm_ros::ModifyPDOVariables::ConstPtr &new_var)
{
    std::string type = new_var->type;

    type = utilities::trim(type);

    switch (int_type_map_[type])
    {
    case 0:

    {
        uint8_t *new_data_ptr = (process_data_buf + slave_id * (num_process_data_out + num_process_data_in) + new_var->index);
        bool value = new_var->bool_value;
        ROS_INFO("New value will be: %d\n", value);
        EC_WRITE_BIT(new_data_ptr, new_var->subindex, value);
        break;
    }

    case 1:

    {
        uint8_t *new_data_ptr = (uint8_t *)(process_data_buf + slave_id * (num_process_data_out + num_process_data_in) + new_var->index);
        uint8_t value = new_var->uint8_value;
        ROS_INFO("New value will be: %d\n", value);
        EC_WRITE_U8(new_data_ptr, value);
        break;
    }

    case 2:

    {
        int8_t *new_data_ptr = (int8_t *)(process_data_buf + slave_id * (num_process_data_out + num_process_data_in) + new_var->index);
        int8_t value = new_var->int8_value;
        ROS_INFO("New value will be: %d\n", value);
        EC_WRITE_S8(new_data_ptr, value);
        break;
    }

    case 3:

    {
        uint16_t *new_data_ptr = (uint16_t *)(process_data_buf + slave_id * (num_process_data_out + num_process_data_in) + new_var->index);
        // printf("O:");
        // for(int j = 0 ; j < 24; j++)
        //     printf(" %2.2x", *(new_var->value[j]);
        // printf("\n");
        uint16_t value = new_var->uint16_value;
        ROS_INFO("New value will be: %d\n", value);
        EC_WRITE_U16(new_data_ptr, value);
        break;
    }

    case 4:
    {
        int16_t *new_data_ptr = (int16_t *)(process_data_buf + slave_id * (num_process_data_out + num_process_data_in) + new_var->index);
        int16_t value = new_var->int16_value;
        ROS_INFO("New value will be: %d\n", value);
        EC_WRITE_S16(new_data_ptr, value);
        break;
    }

    case 5:

    {
        uint32_t *new_data_ptr = (uint32_t *)(process_data_buf + slave_id * (num_process_data_out + num_process_data_in) + new_var->index);
        uint32_t value = new_var->uint32_value;
        ROS_INFO("New value will be: %d\n", value);
        EC_WRITE_U32(new_data_ptr, value);
        break;
    }
    case 6:
    {
        int32_t *new_data_ptr = (int32_t *)(process_data_buf + slave_id * (num_process_data_out + num_process_data_in) + new_var->index);
        int32_t value = new_var->int32_value;
        ROS_INFO("New value will be: %d\n", value);
        EC_WRITE_S32(new_data_ptr, value);
        break;
    }
    case 7:

    {
        uint64_t *new_data_ptr = (uint64_t *)(process_data_buf + new_var->slave_id * (num_process_data_out + num_process_data_in) + new_var->index);
        uint64_t value = new_var->uint64_value;
        EC_WRITE_U64(new_data_ptr, value);
        break;
    }

    case 8:

    {
        int64_t *new_data_ptr = (int64_t *)(process_data_buf + slave_id * (num_process_data_out + num_process_data_in) + new_var->index);
        int64_t value = new_var->int64_value;
        EC_WRITE_S64(new_data_ptr, value);
        break;
    }
    default:
        ROS_ERROR("default handle called (shouldn't): check what you send to the /pdo_listener\n");
        break;
    }
}
void ModifyPDOVariablesListener::init(ros::NodeHandle &n)
{
    //Create  ROS subscriber for the Ethercat RAW data
    modify_pdo_variables_listener_ = n.subscribe("pdo_listener", 1000, &ModifyPDOVariablesListener::modify_pdo_variables_callback, &modify_pdo_variables_listener);
}
