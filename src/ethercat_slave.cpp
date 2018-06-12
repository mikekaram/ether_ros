
#include <iostream>
#include "ethercat_slave.h"
#include "ighm_ros.h"

void EthercatSlave::initialize(std::string slave, ros::NodeHandle& n)
{

    slave_id = slave;
    std::string root_loc = std::string("/ethercat_slaves/") + slave + "/";

    while (!n.getParam(root_loc + "vendor_id", vendor_id))
    {
        ROS_INFO("Waiting the parameter server to initialize\n");
    }
    ROS_INFO("Got param: root_loc + vendor_id = %2.2x\n", vendor_id);

    if (n.getParam(root_loc + "alias", alias))
    {
        ROS_INFO("Got param: root_loc + alias = %d\n", alias);
    }
    else
    {
        ROS_FATAL("Failed to get param 'root_loc + alias'\n");
    }

    if (n.getParam(root_loc + "position", position))
    {
        ROS_INFO("Got param: root_loc + position = %d\n", position);
    }
    else
    {
        ROS_FATAL("Failed to get param 'root_loc + position'\n");
    }

    if (n.getParam(root_loc + "product_code", product_code))
    {
        ROS_INFO("Got param: root_loc + product_code = %2.2x\n", product_code);
    }
    else
    {
        ROS_FATAL("Failed to get param 'root_loc + product_code'\n");
    }

    if (n.getParam(root_loc + "assign_activate", assign_activate))
    {
        ROS_INFO("Got param: root_loc + assign_activate = %2.2x\n", assign_activate);
    }
    else
    {
        ROS_FATAL("Failed to get param 'root_loc + assign_activate'\n");
    }

    if (n.getParam(root_loc + "input_port", input_port))
    {
        ROS_INFO("Got param: root_loc + input_port = %2.2x\n", input_port);
    }
    else
    {
        ROS_FATAL("Failed to get param 'root_loc + input_port'\n");
    }

    if (n.getParam(root_loc + "output_port", output_port))
    {
        ROS_INFO("Got param: root_loc + output_port = %2.2x\n", output_port);
    }
    else
    {
        ROS_FATAL("Failed to get param 'root_loc + output_port'\n");
    }
    ighm_slave = ecrt_master_slave_config(master, alias, position, vendor_id, product_code);
    if (!ighm_slave)
    {
        ROS_FATAL("Failed to get slave configuration.\n");
        exit(1);
    }
    pdo_out = ecrt_slave_config_reg_pdo_entry(ighm_slave, output_port, 1, domain1, NULL);
    if (pdo_out < 0)
    {
        ROS_FATAL("Failed to configure pdo out.\n");
        exit(1);
    }
    ROS_INFO("Offset pdo out is: %d\n", pdo_out);

    pdo_in = ecrt_slave_config_reg_pdo_entry(ighm_slave, input_port, 1, domain1, NULL);
    if (pdo_in < 0)
    {
        ROS_FATAL("Failed to configure pdo in.\n");
        exit(1);
    }
    ROS_INFO("Offset pdo in is: %d\n", pdo_in);

    // configure SYNC signals for this slave
    //For XMC use: 0x0300
    //For Beckhoff FB1111 use: 0x0700
    ecrt_slave_config_dc(ighm_slave, assign_activate, PERIOD_NS, PERIOD_NS / 100, 0, 0);
}

int EthercatSlave::get_pdo_in()
{
    return pdo_in;
}

int EthercatSlave::get_pdo_out()
{
    return pdo_out;
}
