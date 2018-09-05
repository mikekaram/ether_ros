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
   \file services.cpp
   \brief Implements the services used.
   
   Provides services for:
   - Interacting with the EtherCAT Communicator
   - Changing the EtherCAT output PDOs
*/

/*****************************************************************************/

#include "services.h"
#include "ecrt.h"
#include "ethercat_communicator.h"
#include "ighm_ros.h"

/** \fn std::string &ltrim(std::string &str, const std::string &chars = "\t\n\v\f\r ")
    \brief Left trims a string.

    This function trims any character specified in \a chars, which is left of the input
    \a str.

    \param str The input untrimmed string.
    \param chars The characters to trim.
*/
/** \fn std::string &rtrim(std::string &str, const std::string &chars = "\t\n\v\f\r ")
    \brief Right trims a string.

    This function trims any character specified in \a chars, which is right of the input
    \a str.

    \param str The input untrimmed string.
    \param chars The characters to trim.
*/
/** \fn std::string &trim(std::string &str, const std::string &chars = "\t\n\v\f\r ")
    \brief Trims a string from the left and right.

    This function trims any character specified in \a chars, which is right  or left 
    of the input \a str. Calls internally the \a ltrim and \a rtrim functions.
    See more at: http://www.martinbroadhurst.com/how-to-trim-a-stdstring.html
    \see ltrim
    \see rtrim
*/
// trim string from start
std::string &ltrim(std::string &str, const std::string &chars = "\t\n\v\f\r ")
{
    str.erase(0, str.find_first_not_of(chars));
    return str;
}

std::string &rtrim(std::string &str, const std::string &chars = "\t\n\v\f\r ")
{
    str.erase(str.find_last_not_of(chars) + 1);
    return str;
}

std::string &trim(std::string &str, const std::string &chars = "\t\n\v\f\r ")
{
    return ltrim(rtrim(str, chars), chars);
}

bool modify_output_bit(ighm_ros::ModifyOutputBit::Request &req,
                       ighm_ros::ModifyOutputBit::Response &res)
{
    uint8_t *data_ptr = process_data_buf;
    uint8_t *new_data_ptr = (data_ptr + req.slave * (num_process_data_out + num_process_data_in) + req.index);
    pthread_spin_lock(&lock);
    EC_WRITE_BIT(new_data_ptr, req.subindex, req.value);
    pthread_spin_unlock(&lock);
    res.success = "true";
    return true;
}

bool modify_output_sbyte(ighm_ros::ModifyOutputBit::Request &req,
                       ighm_ros::ModifyOutputBit::Response &res)
{
    uint8_t *data_ptr = process_data_buf;
    int8_t *new_data_ptr = (int8_t *)(data_ptr + req.slave * (num_process_data_out + num_process_data_in) + req.index);
    pthread_spin_lock(&lock);
    EC_WRITE_S8(new_data_ptr, req.value);
    pthread_spin_unlock(&lock);
    res.success = "true";
    return true;
}

bool modify_output_uint16(ighm_ros::ModifyOutputUInt16::Request &req,
                          ighm_ros::ModifyOutputUInt16::Response &res)
{
    uint8_t *data_ptr = process_data_buf;
    uint16_t *new_data_ptr = (uint16_t *)(data_ptr + req.slave * (num_process_data_out + num_process_data_in) + req.index);
    pthread_spin_lock(&lock);
    EC_WRITE_U16(new_data_ptr, req.value);
    pthread_spin_unlock(&lock);
    res.success = "true";
    return true;
}

bool modify_output_sint16(ighm_ros::ModifyOutputSInt16::Request &req,
                          ighm_ros::ModifyOutputSInt16::Response &res)
{
    uint8_t *data_ptr = process_data_buf;
    int16_t *new_data_ptr = (int16_t *)(data_ptr + req.slave * (num_process_data_out + num_process_data_in) + req.index);
    pthread_spin_lock(&lock);
    EC_WRITE_S16(new_data_ptr, req.value);
    pthread_spin_unlock(&lock);
    res.success = "true";
    return true;
}

bool modify_output_sint32(ighm_ros::ModifyOutputSInt32::Request &req,
                          ighm_ros::ModifyOutputSInt32::Response &res)
{
    uint8_t *data_ptr = process_data_buf;
    int32_t *new_data_ptr = (int32_t *)(data_ptr + req.slave * (num_process_data_out + num_process_data_in) + req.index);
    pthread_spin_lock(&lock);
    EC_WRITE_S32(new_data_ptr, req.value);
    pthread_spin_unlock(&lock);
    res.success = "true";
    return true;
}
bool ethercat_communicatord(ighm_ros::EthercatCommd::Request &req,
                            ighm_ros::EthercatCommd::Response &res)
{
    bool s;
    req.mode = trim(req.mode);

    if (req.mode == "start")
    {
        s = start_ethercat_communicator();
        res.success = s ? "true" : "false";
        return true;
    }
    else if (req.mode == "stop")
    {
        s = stop_ethercat_communicator();

        res.success = s ? "true" : "false";
        return true;
    }
    else if (req.mode == "thread")
    {

        res.success = ethercat_comm.has_running_thread() ? "true" : "false";
        return true;
    }
    else if (req.mode == "restart")
    {
        if (!stop_ethercat_communicator())
        {
            res.success = "false";
            return true;
        }
        if (!start_ethercat_communicator())
        {
            res.success = "false";
            return true;
        }
        res.success = "true";
        return true;
    }
    else if (req.mode == "clear")
    {
        pthread_spin_lock(&lock);
        memset(process_data_buf, 0, total_process_data); // fill the buffer with zeros
        pthread_spin_unlock(&lock);
        res.success = "true";
        return true;
    }
    else
        return false;
}

bool start_ethercat_communicator()
{
    if (!(bool)ethercat_comm.has_running_thread())
    {
        ethercat_comm.start();
        return true;
    }
    else
        return false;
}

bool stop_ethercat_communicator()
{
    if ((bool)ethercat_comm.has_running_thread())
    {
        ethercat_comm.stop();
        return true;
    }
    else
        return false;
}
