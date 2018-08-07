
#ifndef ETH_COM_LIB_H
#define ETH_COM_LIB_H

#include <iostream>
#include <pthread.h>
#include "ros/ros.h"
#include <sched.h>
void check_domain1_state(void);
void check_master_state(void);
class EthercatCommunicator
{
private:
  pthread_attr_t current_thattr_;
  struct sched_param sched_param_;
  static int cleanup_pop_arg_;
  static pthread_t communicator_thread_;
  static ros::Publisher data_raw_pub_;
  static bool running_thread_;
  static void *run(void *arg);
  static void cleanup_handler(void *arg);
  static void copy_data_to_domain_buf();
  static void publish_raw_data();

public:
  static bool has_running_thread();
  void init(ros::NodeHandle &n);
  void start();
  void stop();
};
#endif /* ETH_COM_LIB_H */
