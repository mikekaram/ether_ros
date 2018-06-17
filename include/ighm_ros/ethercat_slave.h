
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
    void initialize(std::string slave, ros::NodeHandle& n);
    int get_pdo_out();
    int get_pdo_in();
};

#endif /* ETH_SLAVE_LIB_H */
