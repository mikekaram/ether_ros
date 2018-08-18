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
 *  Public License as published by the Free Software Foundation; version 3
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace library. If not, see
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

#ifndef ETH_OUTPUT_DATA_HANDLER_LIB_H
#define ETH_OUTPUT_DATA_HANDLER_LIB_H

#include "ros/ros.h"
#include "ighm_ros/EthercatRawData.h"
class EthercatOutputDataHandler
{
  private:
    ros::Subscriber data_raw_sub_;
    ros::Publisher output_data_pub_;

  public:
    void init(ros::NodeHandle &n);
    void raw_data_callback(const ighm_ros::EthercatRawData::ConstPtr &ethercat_data_raw);
};

#endif /* ETH_OUTPUT_DATA_HANDLER_LIB_H */
