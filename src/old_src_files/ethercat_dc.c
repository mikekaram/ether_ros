/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 ****************************************************************************/

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
#include "utilities.h"
/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY 5000 //define frequency in Hz
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define MEASURE_TIMING 2
#define RUN_TIME 900 // run time in seconds
#if MEASURE_TIMING == 1
    #define SAMPLING_FREQ 10
#endif
/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)
#define NUM_SLAVES 1
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
	(B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/

double ttotal = 0;
uint8_t BLUE_LED_index = 4;
uint8_t knee_angle_index = 3;
uint8_t hip_angle_index = 0;
uint8_t measurement_index = 64;
static pthread_t cyclic_thread;
int run = 1;

/****************************************************************************/
// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

#define ForeLeg    0, 0
// #define DigOutSlavePos   0, 1
// #define CounterSlavePos  0, 2

// #define BECKHOFF_FB1111 0x00000A12, 0x00A986FD
#define XMC_4800 0x00000A12, 0x00000000
// #define IDS_Counter     0x000012ad, 0x05de3052

// offsets for PDO entries
static int pdo_out;
static int pdo_in;
static int off_counter_out;

static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};
char  file_name[100], new_string[100];
static int log_fd;
/*****************************************************************************/

#if MEASURE_TIMING == 1
    
    uint32_t period_ns, exec_ns = 0, latency_ns = 0,
            latency_min_ns[RUN_TIME*SAMPLING_FREQ] = {0}, latency_max_ns[RUN_TIME*SAMPLING_FREQ] = {0},
            period_min_ns[RUN_TIME*SAMPLING_FREQ] = {0}, period_max_ns[RUN_TIME*SAMPLING_FREQ] = {0},
            exec_min_ns[RUN_TIME*SAMPLING_FREQ] = {0}, exec_max_ns[RUN_TIME*SAMPLING_FREQ] = {0};
#elif MEASURE_TIMING == 2
    uint32_t latency_ns[RUN_TIME*FREQUENCY] = {0},
            period_ns[RUN_TIME*FREQUENCY] = {0},
            exec_ns[RUN_TIME*FREQUENCY] = {0}; 
#endif
/*****************************************************************************/
#define handle_error_en(en,msg) \
        do {errno = en; perror(msg); exit(EXIT_FAILURE);} while(0)

/*****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
	struct timespec result;

	if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
		result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
		result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
	} else {
		result.tv_sec = time1.tv_sec + time2.tv_sec;
		result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
	}

	return result;
}

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

	if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/****************************************************************************/


/****************************************************************************/
void signal_handler(int sig)
{
    run = 0;
}
/****************************************************************************/

void *ec_thread(void *arg)
{
    struct timespec start_time, end_time, last_start_time = {},
    break_time,current_time,offset_time = {RUN_TIME,0},wakeup_time;
    uint16_t noise;
    uint8_t pdo_in_end = 5;
    int32_t hip_angle,knee_angle;
    int ret;
    int i = 0;

    
    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeup_time);
    clock_gettime(CLOCK_TO_USE, &break_time);
    break_time = timespec_add(break_time, offset_time);
	do {
		wakeup_time = timespec_add(wakeup_time, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeup_time, NULL);
#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &start_time);
    #if MEASURE_TIMING == 1
            latency_ns = DIFF_NS(wakeup_time, start_time);
            period_ns = DIFF_NS(last_start_time, start_time);
            exec_ns = DIFF_NS(last_start_time, end_time);
    #elif MEASURE_TIMING == 2
            latency_ns[i] = DIFF_NS(wakeup_time, start_time);
            period_ns[i] = DIFF_NS(last_start_time, start_time);
            exec_ns[i] = DIFF_NS(last_start_time, end_time);
    #endif
#endif
        last_start_time = start_time;
#if MEASURE_TIMING == 1
        if (latency_ns > latency_max_ns[i]) {
            latency_max_ns[i] = latency_ns;
        }
        if (latency_ns < latency_min_ns[i]) {
            latency_min_ns[i] = latency_ns;
        }
        if (period_ns > period_max_ns[i]) {
            period_max_ns[i] = period_ns;
        }
        if (period_ns < period_min_ns[i]) {
            period_min_ns[i] = period_ns;
        }
        if (exec_ns > exec_max_ns[i]) {
            exec_max_ns[i] = exec_ns;
        }
        if (exec_ns < exec_min_ns[i]) {
            exec_min_ns[i] = exec_ns;
        }
#endif

		// receive process data
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);
        // noise = process_input_uint16(domain1_pd + off_counter_in,0);

        // hip_angle = process_input_int32(domain1_pd + pdo_in,hip_angle_index, pdo_in_end+1);
        // knee_angle = process_input_int32(domain1_pd + pdo_in,knee_angle_index,  pdo_in_end+0);
        // printf("I: %d , %d \n", hip_angle,knee_angle);
        // printf("I:");
        // for(int j = pdo_in ; j < 38; j++)
        //     printf(" %2.2x", *(domain1_pd + j));
        // printf("\n");
		// check process data state (optional)
		// check_domain_state();

		if (counter) {
			counter--;
		} else { // do this at 10 Hz
#if MEASURE_TIMING == 1
			counter = FREQUENCY/SAMPLING_FREQ;
#elif MEASURE_TIMING == 2
            counter = 0;
#endif
            i++;
			// check for master state (optional)
			// check_master_state();

#if MEASURE_TIMING == 1
            // output timing stats
            // printf("period     %10u ... %10u\n",
            //         period_min_ns, period_max_ns);
            // printf("exec       %10u ... %10u\n",
            //         exec_min_ns, exec_max_ns);
            // printf("latency    %10u ... %10u\n",
            //         latency_min_ns, latency_max_ns);
            
            period_max_ns[i] = 0;
            period_min_ns[i] = 0xffffffff;
            exec_max_ns[i] = 0;
            exec_min_ns[i] = 0xffffffff;
            latency_max_ns[i] = 0;
            latency_min_ns[i] = 0xffffffff;
#endif
            blink = !blink;


			// calculate new process data
		}

		// write process data
		// modify_output_bit(domain1_pd + pdo_out, BLUE_LED_index,blink);
        modify_output_bit(domain1_pd+pdo_out, measurement_index,1);
        // printf("O:");
        // for(int j = 0 ; j < 24; j++)
        //     printf(" %2.2x", *(domain1_pd + j));
        // printf("\n");

		// write application time to master
		clock_gettime(CLOCK_TO_USE, &current_time);
		ecrt_master_application_time(master, TIMESPEC2NS(current_time));

		if (sync_ref_counter) {
			sync_ref_counter--;
		} else {
			sync_ref_counter = 5; // sync every 5 cycles
			ecrt_master_sync_reference_clock(master);
		}
		ecrt_master_sync_slave_clocks(master);

		// send process data
		ecrt_domain_queue(domain1);
		ecrt_master_send(master);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &end_time);
#endif
        clock_gettime(CLOCK_TO_USE, &current_time);
        // printf("Current time is: %ld and Break Time is: %ld\n",current_time.tv_sec, break_time.tv_sec);
    }
    
    while(DIFF_NS(current_time,break_time) > 0);
    
    // write the statistics to file
#if MEASURE_TIMING == 1
    for(i=0;i<RUN_TIME*SAMPLING_FREQ;i++){
        snprintf(new_string,100,"%10u , %10u , 10u , %10u , 10u , %10u\n",
                period_min_ns[i], period_max_ns[i],exec_min_ns[i], exec_max_ns[i],
                latency_min_ns[i], latency_max_ns[i]);
        fprintf(file,"%s",string_file);   
    }
#elif MEASURE_TIMING == 2
    for(i=0;i<RUN_TIME*FREQUENCY;i++){
        if(i%10000 == 0)
            printf("Current line written is: %d\n",i);
        snprintf(new_string,sizeof(new_string),"%10u , %10u , %10u\n",
                period_ns[i],exec_ns[i],latency_ns[i]);
        if((uint32_t)insist_write(log_fd, new_string, strlen(new_string)) != strlen(new_string)) {
            perror("ec_thread: insist_write");
            exit(1);
        }           
    }
#endif
    if(close(log_fd)) {
        perror("ec_thread: close log fd");
        exit(1);
    }
    exit(0);
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;
    
    int ret;

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		perror("mlockall failed");
		return -1;
	}

    master = ecrt_request_master(0);
    if (!master)
        return -1;
    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
        return -1;
    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, ForeLeg, XMC_4800);
    if (!sc){
    	fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }


    // if (!(sc = ecrt_master_slave_config(master,
    //                 DigOutSlavePos, Beckhoff_EL2008))) {
    //     fprintf(stderr, "Failed to get slave configuration.\n");
    //     return -1;
    // }

    pdo_out = ecrt_slave_config_reg_pdo_entry(sc,
            0x7000, 1, domain1, NULL);
    if (pdo_out < 0)
        return -1;
    printf("Offset pdo out is: %d\n",pdo_out);

	// if (!(sc = ecrt_master_slave_config(master,
	// 				CounterSlavePos, IDS_Counter))) {
 //        fprintf(stderr, "Failed to get slave configuration.\n");
 //        return -1;
	// }

	pdo_in = ecrt_slave_config_reg_pdo_entry(sc,
			0x6000, 1, domain1, NULL);
	if (pdo_in < 0)
        return -1;
    printf("Offset pdo in is: %d\n",pdo_in);
	// off_counter_out = ecrt_slave_config_reg_pdo_entry(sc,
	// 		0x7020, 1, domain1, NULL);
	// if (off_counter_out < 0)
 //        return -1;

    // configure SYNC signals for this slave
    // shift time = 4400000
    //For XMC use: 0x0300
    //For Beckhoff FB1111 use: 0x0700
	// ecrt_slave_config_dc(sc, 0x0700, PERIOD_NS, PERIOD_NS/10, 0, 0);
    ecrt_slave_config_dc(sc, 0x0300, PERIOD_NS, PERIOD_NS/100, 0, 0);
    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

// ************************************************
    /* Open log file */
    sprintf(file_name,"./outputs/measure_test_2/nort_ectest_dc_res_time_%d_%d_Hz_%d_s.csv",NUM_SLAVES,FREQUENCY,RUN_TIME);
    mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH;
    int flags = O_WRONLY | O_CREAT | O_TRUNC;
    log_fd = open(file_name, flags, mode); 
    if(log_fd < 0){
        perror("main: open fd");
        exit(1);
    }


// ************************************************
    /* Create cyclic RT-thread */
    const struct sched_param param = { .sched_priority = 80 };
    struct sched_param act_param = {};
    int act_policy;
    pthread_attr_t thattr;
    struct timespec current_time;
    if(pthread_attr_init(&thattr)){
        perror("Attribute init\n");
        exit(1);
    }
    if(pthread_attr_setdetachstate(&thattr, PTHREAD_CREATE_DETACHED)){
        perror("Attribute set detach state\n");
        exit(1);
    }
    if(pthread_attr_setinheritsched(&thattr, PTHREAD_EXPLICIT_SCHED)){
        perror("Attribute set inherit schedule\n");
        exit(1);
    }
    if(pthread_attr_setschedpolicy(&thattr, SCHED_FIFO)){
        perror("Attribute set schedule policy\n");
        exit(1);
    }
    // int prio_max = sched_get_priority_max(SCHED_FIFO);
    // int prio_min = sched_get_priority_min(SCHED_FIFO);
    // printf("Maximum schedulable priority is: %d and minimum is: %d\n",prio_max,prio_min);
    ret = pthread_attr_setschedparam(&thattr,&param);
    if(ret != 0){
            handle_error_en(ret,"pthread_attr_setschedparam");
    }
    ret = pthread_attr_getschedparam(&thattr,&act_param);
    if(ret != 0){
            handle_error_en(ret,"pthread_attr_getschedparam");
    }
    ret = pthread_attr_getschedpolicy(&thattr,&act_policy);
    if(ret != 0){
            handle_error_en(ret,"pthread_attr_getschedpolicy");
    }
    printf("Actual pthred attribute values are: %d , %d\n", act_policy, act_param.sched_priority);

    ret = pthread_create(&cyclic_thread, &thattr, &ec_thread, NULL);
    if (ret != 0) {
        handle_error_en(ret,"pthread_create");
    }
// ************************************************
	/* Wait for cyclic RT-thread to finish */
    printf("Starting cyclic thread.\n");
    while(run)
	    sched_yield();
    return 0;
}

/****************************************************************************/
