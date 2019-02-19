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
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h>

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

/** Task frequency in Hz. */
#define FREQUENCY  (5000)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */

/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC (1000000000)
#define CLOCK_TO_USE CLOCK_REALTIME
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)
#define SetBit(A,k)     ( A[(k/8)] |= (1 << (k%8)) )
#define ClearBit(A,k)   ( A[(k/8)] &= ~(1 << (k%8)) )
#define NUM_SLAVES 1
#define MEASURE_TIMING
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
	(B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
#define RUN_TIME 60 // run time in seconds
const struct timespec cycletime = {0, PERIOD_NS};
// int nLoops = 10000;
double ttotal = 0;
uint8_t write_index = 64;


/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_ana_in = NULL;
static ec_slave_config_state_t sc_ana_in_state = {};

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

#define BusCouplerPos    0, 0
#define DigOutSlavePos   0, 1
// #define CounterSlavePos  0, 2

#define Beckhoff_EK1100 0x00000A12, 0x00000000
#define Beckhoff_EL2008 0x00000A12, 0x00000000

// offsets for PDO entries
static int off_dig_out;
static int off_counter_in;
static int off_counter_out;

// const static ec_pdo_entry_reg_t domain1_regs[] = {
//     {AnaInSlavePos,  Beckhoff_EL3102, 0x3101, 1, &off_ana_in_status},
//     {AnaInSlavePos,  Beckhoff_EL3102, 0x3101, 2, &off_ana_in_value},
//     {AnaOutSlavePos, Beckhoff_EL4102, 0x3001, 1, &off_ana_out},
//     {DigOutSlavePos, Beckhoff_EL2032, 0x3001, 1, &off_dig_out},
//     {}
// };

static unsigned int counter = 0;
static unsigned int blink = 0;
int sync_ref_counter = 0;
struct timeval tts,ttf;
char * s, *d;
FILE * f;
/*****************************************************************************/

// Analog in --------------------------

// static ec_pdo_entry_info_t el3102_pdo_entries[] = {
//     {0x3101, 1,  8}, // channel 1 status
//     {0x3101, 2, 16}, // channel 1 value
//     {0x3102, 1,  8}, // channel 2 status
//     {0x3102, 2, 16}, // channel 2 value
//     {0x6401, 1, 16}, // channel 1 value (alt.)
//     {0x6401, 2, 16}  // channel 2 value (alt.)
// };

// static ec_pdo_info_t el3102_pdos[] = {
//     {0x1A00, 2, el3102_pdo_entries},
//     {0x1A01, 2, el3102_pdo_entries + 2}
// };

// static ec_sync_info_t el3102_syncs[] = {
//     {2, EC_DIR_OUTPUT},
//     {3, EC_DIR_INPUT, 2, el3102_pdos},
//     {0xff}
// };

// // Analog out -------------------------

// static ec_pdo_entry_info_t el4102_pdo_entries[] = {
//     {0x3001, 1, 16}, // channel 1 value
//     {0x3002, 1, 16}, // channel 2 value
// };

// static ec_pdo_info_t el4102_pdos[] = {
//     {0x1600, 1, el4102_pdo_entries},
//     {0x1601, 1, el4102_pdo_entries + 1}
// };

// static ec_sync_info_t el4102_syncs[] = {
//     {2, EC_DIR_OUTPUT, 2, el4102_pdos},
//     {3, EC_DIR_INPUT},
//     {0xff}
// };

// // Digital out ------------------------

// static ec_pdo_entry_info_t el2004_channels[] = {
//     {0x3001, 1, 1}, // Value 1
//     {0x3001, 2, 1}, // Value 2
//     {0x3001, 3, 1}, // Value 3
//     {0x3001, 4, 1}  // Value 4
// };

// static ec_pdo_info_t el2004_pdos[] = {
//     {0x1600, 1, &el2004_channels[0]},
//     {0x1601, 1, &el2004_channels[1]},
//     {0x1602, 1, &el2004_channels[2]},
//     {0x1603, 1, &el2004_channels[3]}
// };

// static ec_sync_info_t el2004_syncs[] = {
//     {0, EC_DIR_OUTPUT, 4, el2004_pdos},
//     {1, EC_DIR_INPUT},
//     {0xff}
// };

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    // if (ds.working_counter != domain1_state.working_counter) {
    //     printf("Domain1: WC %u.\n", ds.working_counter);
    // }
    // if (ds.wc_state != domain1_state.wc_state) {
    //     printf("Domain1: State %u.\n", ds.wc_state);
    // }

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_ana_in, &s);

    if (s.al_state != sc_ana_in_state.al_state) {
        printf("AnaIn: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_ana_in_state.online) {
        printf("AnaIn: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_ana_in_state.operational) {
        printf("AnaIn: %soperational.\n", s.operational ? "" : "Not ");
    }

    sc_ana_in_state = s;
}
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
void set_output_bit (uint8_t * data_ptr, uint8_t index)
{
//    uint8 *data_ptr;
//    data_ptr = ec_slave[slave_no].outputs;
   /* Move pointer to correct module index*/
//    data_ptr += module_index * 2;
   /* Read value byte by byte since all targets can't handle misaligned
 addresses
    */
//   *data_ptr++ = (value >> 0) & 0xFF;
//   *data_ptr++ = (value >> 8) & 0xFF;
//   printf("Value of shift is: %d, array index is: %d and bit index is: %d\n",value,value/8,value%8);
    SetBit(data_ptr,index);
//    if(value == 0)
//       ClearBit(data_ptr,71);
//    else
//    {
//       printf("Clearing %d bit\n",value-1);
//       ClearBit(data_ptr,value-1);
//    }

}
uint16_t process_input_int16(uint8_t * data_ptr, uint8_t index)
{
    // uint8 * data_ptr;
    uint16_t return_value = 0x0000;
    // data_ptr = ec_slave[slave_no].inputs;
    /* Move pointer to correct module index*/
    // data_ptr += module_index * 2;
    return_value = data_ptr[index+1];
    return_value = return_value << 8;
    return_value |= data_ptr[index];
//    return_value = ((data_ptr[index]>>8)&0xFFFF) | (data_ptr[index+1]<<8&0xFFFF);
//    printf("Return Value is: %d\n",return_value);
    return return_value;
}

void cyclic_task()
{
    struct timespec wakeupTime, time;
    uint16_t noise;
#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime = {},breakTime,currentTime,offsetTime = {RUN_TIME,0};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0;
#endif
    set_output_bit(domain1_pd + off_dig_out, write_index);
    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);
    clock_gettime(CLOCK_TO_USE, &breakTime);
    breakTime = timespec_add(breakTime, offsetTime);
	do {
		wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns) {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns) {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns) {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns) {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns) {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns) {
            exec_min_ns = exec_ns;
        }
#endif
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    uint16_t noise = process_input_int16(domain1_pd + off_counter_in,0);
    // printf("I: %2.2d \n", noise);
    // check process data state
    check_domain1_state();

    if (counter) {
        counter--;
    }
    else { // do this at 100 Hz
        counter = FREQUENCY/10;

        // calculate new process data
        // blink = !blink;

        // check for master state (optional)
        // check_master_state();

        // check for slave configuration state(s) (optional)
        // check_slave_config_states();

        #ifdef MEASURE_TIMING
            // output timing stats
            // printf("period     %10u ... %10u\n",
            //         period_min_ns, period_max_ns);
            // printf("exec       %10u ... %10u\n",
            //         exec_min_ns, exec_max_ns);
            // printf("latency    %10u ... %10u\n",
            //         latency_min_ns, latency_max_ns);
            sprintf(d,"%10u , %10u , ",
                    period_min_ns, period_max_ns);
            fprintf(f,"%s",d);
            sprintf(d,"%10u , %10u , ",
                    exec_min_ns, exec_max_ns);
            fprintf(f,"%s",d);
            sprintf(d,"%10u , %10u\n",
                    latency_min_ns, latency_max_ns);
            fprintf(f,"%s",d);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
        #endif

			// calculate new process data
    }

    // write process data
    set_output_bit(domain1_pd + off_dig_out, write_index);
    // write application time to master
    clock_gettime(CLOCK_TO_USE, &time);
    ecrt_master_application_time(master, TIMESPEC2NS(time));

    // if (sync_ref_counter) {
    //     sync_ref_counter--;
    // } else {
    //     sync_ref_counter = 1; // sync every cycle
    //     ecrt_master_sync_reference_clock(master);
    // }
    // ecrt_master_sync_slave_clocks(master);

    // send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

    #ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
    #endif
    clock_gettime(CLOCK_TO_USE, &currentTime);
    }while(DIFF_NS(currentTime,breakTime) > 0);
}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;
    struct timespec wakeup_time;
    int ret = 0;
    master = ecrt_request_master(0);
    if (!master)
        return -1;
    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
        return -1;
    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK1100);
    if (!sc){
    	fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    // if (!(sc = ecrt_master_slave_config(master,
    //                 DigOutSlavePos, Beckhoff_EL2008))) {
    //     fprintf(stderr, "Failed to get slave configuration.\n");
    //     return -1;
    // }

    off_dig_out = ecrt_slave_config_reg_pdo_entry(sc,
            0x7000, 1, domain1, NULL);
    if (off_dig_out < 0)
        return -1;
    printf("Offset digital out is: %d\n",off_dig_out);

	// if (!(sc = ecrt_master_slave_config(master,
	// 				CounterSlavePos, IDS_Counter))) {
 //        fprintf(stderr, "Failed to get slave configuration.\n");
 //        return -1;
	// }

	off_counter_in = ecrt_slave_config_reg_pdo_entry(sc,
			0x6000, 1, domain1, NULL);
	if (off_counter_in < 0)
        return -1;
    printf("Offset counter in is: %d\n",off_counter_in);

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

    /* Set priority */

    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, -19)) {
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));
    }
    struct sched_param param;
    int high_priority = 80;
    param.sched_priority = high_priority;

    if(sched_setscheduler(pid,SCHED_FIFO,&param)<0){
        perror("Failed to set scheduler priority");
        return -1;
    }
    /* Lock memory */

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    stack_prefault();
// ************************************************
    s=(char *)malloc(100*sizeof(char));
    d=(char *)malloc(100*sizeof(char));
    sprintf(s,"./outputs/dc_vs_sm/ectest_res_time_%d_%d_Hz.csv",NUM_SLAVES,FREQUENCY);

    f=fopen(s,"w");

// ************************************************

    printf("Starting RT task with dt=%u Hz.\n", FREQUENCY);

    cyclic_task();

    fclose(f);
        // printf("%d\n",i);
        // sprintf(d,"%f\n",ttotal);
        // fprintf(f,"%s",d);
}

/****************************************************************************/
