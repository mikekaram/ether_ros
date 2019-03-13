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
   \file pdo_in_publisher.h
   \brief Header file for the PDOInPublisher class.
*/

/*****************************************************************************/

#ifndef PDO_IN_PUB_LIB_H
#define PDO_IN_PUB_LIB_H

#include "ros/ros.h"
#include "ether_ros/PDORaw.h"

/** \class PDOInPublisher
    \brief The Ethercat Input Data Handler class.

    Used for trasforming the "raw" indexed data from
    the \a /pdo_raw topic, sent by the Ethercat
    Communicator, to values of variables, and stream them
    to the \a /pdo_in_slave_{slave_id} topic.
*/
class PDOInPublisher
{
    private:
      ros::Subscriber pdo_raw_sub_;
      ros::Publisher * pdo_in_pub_;
/** \fn void init(ros::NodeHandle &n)
    \brief Initialization Method.

    Used for initializing the PDOInPublisher object. It's basically
    the main method in the class, which initializes the listener to the afore
    mentioned topic.
    \param n The ROS Node Handle
*/
/** \fn void pdo_raw_callback(const ether_ros::PDORaw::ConstPtr &pdo_raw)
    \brief Raw Data Callback

    This method, is called when there are data in the \a /pdo_raw topic.
    Should the EtherCAT application change, this callback must change also.
    Implements the basic functionality of the class, to transform the "raw" data
    into variable values and pipe them into another topic.
    \param pdo_raw A copy of the actual data sent to the topic \a /pdo_raw.
*/
    public:
      void init(ros::NodeHandle &n);
      void pdo_raw_callback(const ether_ros::PDORaw::ConstPtr &pdo_raw);
};

#endif /* PDO_IN_PUB_LIB_H */
