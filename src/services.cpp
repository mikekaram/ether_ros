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
#include "ether_ros.h"
#include "utilities.h"


bool ethercat_communicatord(ether_ros::EthercatCommd::Request &req,
                            ether_ros::EthercatCommd::Response &res)
{
    bool s;
    req.mode = utilities::trim(req.mode);

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
