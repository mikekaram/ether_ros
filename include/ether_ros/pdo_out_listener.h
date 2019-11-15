
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
   \file pdo_out_listener.h
   \brief Header file for the PDOOutListener class.
*/

/*****************************************************************************/

#ifndef MOD_PDO_VAR_LIS_LIB_H
#define MOD_PDO_VAR_LIS_LIB_H

#include "ros/ros.h"
#include "ether_ros/ModifyPDOVariables.h"
#include <map>

/** \class PDOOutListener
    \brief The Ethercat Input Data Handler class.

    Used for trasforming the "raw" indexed data from
    the \a /pdo_raw topic, sent by the Ethercat
    Communicator, to values of variables, and stream them
    to the \a /pdo_in_slave_{slave_id} topic.
*/
class PDOOutListener
{
  private:
    ros::Subscriber pdo_out_listener_;
    std::map<std::string, int> int_type_map_ = {
        {"bool", 0},
        {"uint8", 1},
        {"int8", 2},
        {"uint16", 3},
        {"int16", 4},
        {"uint32", 5},
        {"int32", 6},
        {"uint64", 7},
        {"int64", 8}
    };
    /** \fn void init(ros::NodeHandle &n)
    \brief Initialization Method.

    Used for initializing the PDOInPublisher object. It's basically
    the main method in the class, which initializes the listener to the afore
    mentioned topic.
    \param n The ROS Node Handle
*/
    /** \fn void pdo_out_callback(const ether_ros::ModifyPDOVariables::ConstPtr &new_var);

    This method, is called when there are data in the \a /modify_pdo_var topic.
    Should the EtherCAT application change, this callback must change also.
    Implements the basic functionality of the class, to transform the "raw" data
    into variable values and pipe them into another topic.
    \param pdo_raw A copy of the actual data sent to the topic \a /pdo_raw.
*/
    /** \fn void modify_pdo_variable(int slave_id, const ether_ros::ModifyPDOVariables::ConstPtr &new_var)
    \brief Actual implementation of modifying a pdo variable in a single slave's PDOs

    This method, is called when there are data in the \a /modify_pdo_var topic.
    Should the EtherCAT application change, this callback must change also.
    Implements the basic functionality of the class, to transform the "raw" data
    into variable values and pipe them into another topic.
    \param pdo_raw A copy of the actual data sent to the topic \a /pdo_raw.
*/
    public : void init(ros::NodeHandle & n);
    void pdo_out_callback(const ether_ros::ModifyPDOVariables::ConstPtr &new_var);
    void modify_pdo_variable(int slave_id, const ether_ros::ModifyPDOVariables::ConstPtr &new_var);
};

#endif /* MOD_PDO_VAR_LIS_LIB_H */

