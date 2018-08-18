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

#ifndef ETH_SLAVE_LIB_H
#define ETH_SLAVE_LIB_H

#include <string>
#include "ecrt.h"
#include "ros/ros.h"

class EthercatSlave
{
  private:
    int vendor_id_;
    std::string slave_id_;
    int product_code_;
    int assign_activate_;
    int position_;
    int alias_;
    int input_port_;
    int output_port_;
    ec_slave_config_t *ighm_slave_; //pointer to the basic slave struct in ighm
    int pdo_in_;
    int pdo_out_;

  public:
    void init(std::string slave, ros::NodeHandle& n);
    int get_pdo_out();
    int get_pdo_in();
};

#endif /* ETH_SLAVE_LIB_H */
