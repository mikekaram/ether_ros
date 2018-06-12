
#ifndef IGHM_ROS_LIB_H
#define IGHM_ROS_LIB_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sched.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <stddef.h>
#include "ecrt.h"
#include "ethercat_slave.h"

// Application parameters
#define CLOCK_TO_USE CLOCK_MONOTONIC
// #define MEASURE_TIMING 2
// #define RUN_TIME 60 // run time in seconds
#if MEASURE_TIMING == 1
#define SAMPLING_FREQ 10
#endif
/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)
#define NUM_SLAVES 1
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

#define handle_error_en(en, msg) \
    do                           \
    {                            \
        errno = en;              \
        ROS_FATAL(msg);          \
        exit(EXIT_FAILURE);      \
    } while (0)

/****************************************************************************/

typedef struct slave_struct
{
    std::string slave_name;
    int id;
    EthercatSlave slave;
} slave_struct;

extern slave_struct ethercat_slaves[NUM_SLAVES];
extern uint8_t *domain1_pd;
extern int log_fd;
extern ec_master_t *master;
extern ec_master_state_t master_state;
extern ec_domain_t *domain1;
extern ec_domain_state_t domain1_state;
extern ros::NodeHandle n;
extern pthread_spinlock_t *lock;

int main(int argc, char **argv);

#endif /* IGHM_ROS_LIB_H */
