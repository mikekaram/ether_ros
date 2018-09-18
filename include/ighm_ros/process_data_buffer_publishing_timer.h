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
   \file pdo_out_publisher.h
   \brief Header file for the PDOOutPublisher class.
*/

/*****************************************************************************/

#ifndef PDB_PUB_TIMER_LIB_H
#define PDB_PUB_TIMER_LIB_H

#include "ros/ros.h"

/** \class PDOOutPublisher
    \brief The Process Data Objects Publisher class.

    Used for trasforming the "raw" indexed data from
    the \a /pdo_raw topic, sent by the Ethercat
    Communicator, to values of variables, and stream them
    to the \a /pdo_out topic.
*/
class ProcessDataBufferPublishingTimer
{
  private:
    ros::Publisher process_data_buffer_pub_;
    uint8_t * data_ptr_;
    ros::Timer pdo_out_timer_;

    /** \fn void init(ros::NodeHandle &n)
    \brief Initialization Method.

    Used for initializing the PDOOutPublisher object. It's basically
    the main method in the class, which initializes the listener to the afore
    mentioned topic.
    \param n The ROS Node Handle
*/
    /** \fn void pdo_raw_callback(const ighm_ros::PDORaw::ConstPtr &pdo_raw)
    \brief Process Data Objects Callback

    This method, is called when there are data in the \a /pdo_raw topic.
    Should the EtherCAT application change, this callback must change also.
    Implements the basic functionality of the class, to transform the "raw" data
    into variable values and pipe them into another topic.
    \param pdo_raw A copy of the actual data sent to the topic \a /pdo_raw.
*/
  public:
    void init(ros::NodeHandle &n);
    void timer_callback(const ros::TimerEvent &event);
};

#endif /* PDB_PUB_TIMER_LIB_H */
