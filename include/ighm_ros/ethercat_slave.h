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
   \file ethercat_slave.h
   \brief Header file for the EthercatSlave class.
*/

/*****************************************************************************/

#ifndef ETH_SLAVE_LIB_H
#define ETH_SLAVE_LIB_H

#include <string>
#include "ecrt.h"
#include "ros/ros.h"

/** \class EthercatSlave
    \brief The Ethercat Slave class.

    Used for having all the ethercat slave related variables,
    fetched from the correspondent yaml file, in a single entity.
*/
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
    int32_t sync0_shift_;

  public:
    /** \fn void init(std::string slave, ros::NodeHandle& n)
    \brief Initialization Method.

    Used for initializing the EthercatSlave entity. It's basically
    the main method in the class.
*/
    /** \fn int get_pdo_out()
    \brief Getter Method.

    Used for getting the number of bytes of the output PDO of the single slave.
*/
    /** \fn int get_pdo_in()
    \brief Getter Method.

    Used for getting the number of bytes of the input PDO of the single slave.
*/
    void init(std::string slave, ros::NodeHandle &n);
    int get_pdo_out();
    int get_pdo_in();
    ec_slave_config_t *get_slave_config();
};

#endif /* ETH_SLAVE_LIB_H */
