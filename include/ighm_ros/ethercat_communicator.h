
#ifndef ETH_COM_LIB_H
#define ETH_COM_LIB_H

#include <iostream>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
void check_domain1_state(void);
void check_master_state(void);
class EthercatCommunicator
{
private:
  pthread_attr_t current_thattr_;
  bool has_running_thread_;
  struct sched_param sched_param_;
  uint8_t * process_data_buf_;
  pthread_t communicator_thread_;
  static void *run(void *arg);
  static void cleanup_handler(void *arg);

public:
  EthercatCommunicator();
  bool has_running_thread();
  void init();
  void start();
  void stop();
};
#endif /* ETH_COM_LIB_H */
