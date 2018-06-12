
#ifndef ETH_SLAVE_LIB_H
#define ETH_SLAVE_LIB_H

#include <string>
#include "ecrt.h"

class EthercatSlave
{
  private:
    int vendor_id;
    std::string slave_id;
    int product_code;
    int assign_activate;
    int position;
    int alias;
    int input_port;
    int output_port;
    ec_slave_config_t *ighm_slave; //pointer to the basic slave struct in ighm
    int pdo_in;
    int pdo_out;

  public:
    void initialize(std::string slave);
    int get_pdo_out();
    int get_pdo_in();
};

#endif /* ETH_SLAVE_LIB_H */
