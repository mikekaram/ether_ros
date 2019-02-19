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

#ifndef PDO_OUT_PUB_TIMER_LIB_H
#define PDO_OUT_PUB_TIMER_LIB_H

#include "ros/ros.h"

/** \class PDOOutPublisher
    \brief The Process Data Objects Publisher class.

    Used for streaming the \a pdo_out data inside the \a process_data_buffer to
    the \a /pdo_out_timer topic at a certain rate. It's been created for logging and debugging
    reasons.
*/
class PDOOutPublisherTimer
{
  private:
    ros::Publisher pdo_out_pub_;
    uint8_t * data_ptr_;
    ros::Timer pdo_out_timer_;

    /** \fn void init(ros::NodeHandle &n)
    \brief Initialization Method.

    Used for initializing the PDOOutPublisherTimer object. It's basically
    the main method in the class, which initializes the listener to the afore
    mentioned topic.
    \param n The ROS Node Handle
*/
    /** \fn void timer_callback(const ros::TimerEvent &event)
    \brief Timer Callback

    This method, is called when the timer fires.
    Should the EtherCAT application change, this callback must change also.
    Implements the basic functionality of the class, to copy the \a pdo_out data
    from the \a process_data_buffer and pipe them into another topic.
    \param event The fired timer event.
*/
  public:
    void init(ros::NodeHandle &n);
    void timer_callback(const ros::TimerEvent &event);
};

#endif /* PDO_OUT_PUB_TIMER_LIB_H */
