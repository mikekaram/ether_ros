
#ifndef ETH_COM_LIB_H
#define ETH_COM_LIB_H

#include <iostream>
#include <pthread.h>
class EthercatCommunicator
{
    private:
      static int cleanup_pop_arg_ = 0;
      int cancel_state_;
      pthread_attr_t current_thattr_;
      struct sched_param sched_param_;
      pthread_t * communicator_thread_;
      void run();
      static void cleanup_handler(void *arg);

    public:
      void EthercatCommunicator();
      std::bool has_running_thread();
      void init();
      void start();
      void stop();
};

#endif /* ETH_COM_LIB_H */
