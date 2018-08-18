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
 *  Public License as published by the Free Software Foundation; version 3
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace library. If not, see
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

/** \file
 *
 * IgH EtherCAT Master module main in ROS.
 *
 * \defgroup IgHEMM_ROS EtherCAT module main
 *
 * IgH Master EtherCAT module main for realtime communication with EtherCAT slaves. 
 * This interface is designed for realtime modules, running in the ROS environment 
 * and want to use EtherCAT. There are classes and functions to get the Input and Output PDOs
 * in a realtime context. Before trying to understand the source code of the project,
 * please consider to read and understand the IgH Master Documentation located at: 
 * https://www.etherlab.org/download/ethercat/ethercat-1.5.2.pdf. Finally try to 
 * understand the API provided in the /opt/etherlab/include/ecrt.h file.
 *
 *
 * Changes in version 0.3:
 *
 * - Created a frontend UI client in Python, for interacting with the Services API.
 * Features include:
 * Start/Stop function of the EtherCAT Communicator
 * Change the Output PDOs on the run
 * Run script with the aproppriate Service API calls
 * 
 * 
 * Changes in version 0.2:
 *
 * - Added features and bug fixes including:
 *   First realtime characteristics added (debate: FIFO vs DEADLINE scheduling?)
 *   Handling of the EtherCAT communicator module: Start/Stop/Restart API
 * 
 * - Added processing for the /ethercat_data_raw topic and created:
 *   Service API for Output PDO handling and topic streaming: /ethercat_output_data
 *   Service API for Input PDO topic streaming: /ethercat_data_slave_x
 * 
 * - Added synchronization primitives (spinlocks) for the concurrent threads accessing the EtherCAT buffer.
 * - Added more source files, ethercat communicator, ethercat_slave, ethercat_input_data_handler and 
 * ethercat_output_data_handler. Created external objects to use the appropriate classes and functions.
 * 
 * 
 * Version 0.1:
 *
 * - Created the first bare communication layer in the ROS environment. Many bugs and 
 * deficiencies including: Non realtime characteristics, no API for Output PDO handling and topic
 * streaming, no handling of the EtherCAT communicator module, no Service API for Input PDO topic streaming.
 * - One topic streaming: /ethercat_data_raw
 *
 * @{
 */

/*****************************************************************************/

#include "utilities.h"
#include "services.h"
#include "ighm_ros.h"
#include "ethercat_communicator.h"

/*****************************************************************************/

/************************GLOBAL VARIABLES ************************************/

uint8_t *domain1_pd; /**< Global buffer for the actual communication with the IgH Master Module. */
uint8_t *process_data_buf; /**< Global buffer for safe concurrent accesses from the output PDOs services and the EtherCAT Communicator. \see ethercat_comm */
size_t total_process_data; /**< Total number of process data (PD) (bytes). */
size_t num_process_data_in; /**< Number of input PD per slave (bytes). Assumes that the EtherCAT application is the same for every slave. */
size_t num_process_data_out; /**< Number of output PD per slave (bytes). Assumes that the EtherCAT application is the same for every slave.*/
int log_fd; /**< File descriptor used for logging, provided that measure_timing is enabled. Could be deprecated in a next version (see kernelshark) */
ec_master_t *master; /**< The main master struct. Used for communication with the IgH Master Module. */
ec_master_state_t master_state; /**< The master state struct. Used to examine the current state (Links Up/Down, AL states) of the Master.  */
ec_master_info_t master_info; /**< The master info struct. Used to know the slaves responding to the Master. */
ec_domain_t *domain1; /**< The main domain variable. Used to send and receive the datagrams. */
ec_domain_state_t domain1_state; /**< The domain state struct. Used to examine the current state (Working counter, DL states) of the domain.   */
slave_struct *ethercat_slaves; /**< The main slave struct, used by our program to contain all the useful info of every slave. */
pthread_spinlock_t lock; /**< The shared spinlock, used by every thread whick modifies the process_data_buf. \see process_data_buf */
EthercatCommunicator ethercat_comm; /**< The barebone object of our application. Used for realtime communication (Tx/Rx) with the EtherCAT slaves.
                                            Doesn't change the output PDOs. Basic state machine: 
                                            -  Receive the new PDOs in domain1_pd from the IgH Master Module (and then to EtherCAT slaves)
                                            - Move to the domain_pd the output data of process_data_buf, safely
                                            - Publish the "raw" data (not linked to EtherCAT variables) in PDOs from the domain1_pd
                                            - Send the new PDOs from domain1_pd to the IgH Master Module (and then to EtherCAT slaves)*/
EthercatInputDataHandler ethercat_input_data_handler; /**< Main object for publishing to the /ethercat_data_slave_x the values of the EtherCAT input variables.
                                                        Maps indeces to variables. */
EthercatOutputDataHandler ethercat_output_data_handler; /**< Main object for publishing to the /ethercat_data_out the values of the EtherCAT output variables.
                                                        Maps indeces to variables. */
int FREQUENCY; /**< Frequency of the realtime thread: EtherCAT Communicator */
int RUN_TIME; /**< Total run time of the realtime thread: EtherCAT Communicator */
int PERIOD_NS; /**< Handy variable induced from the Frequency variable. \see FREQUENCY */

/****************************************************************************/

int main(int argc, char **argv)
{
    int ret;
    std::string slave_names[4] = {"front_left_leg", "front_right_leg", "back_right_leg", "back_left_leg"};

    ros::init(argc, argv, "ighm_ros");

    ros::NodeHandle n;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        ROS_FATAL("mlockall failed");
        exit(1);
    }

    ret = pthread_spin_init(&lock, PTHREAD_PROCESS_SHARED); // It doesn't matter for our application what pshared value we use
    if (ret != 0)
    {
        handle_error_en(ret, "pthread_spin_lock_init");
    }

    master = ecrt_request_master(0);
    if (!master)
    {
        ROS_FATAL("Failed to get master.\n");
        exit(1);
    }

    ret = ecrt_master(master, &master_info);
    if (ret != 0)
    {
        handle_error_en(ret, "ecrt_master_info");
    }
    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
        ROS_FATAL("Failed to create domain.\n");
        exit(1);
    }
    if (n.getParam("/ethercat_slaves/frequency", FREQUENCY))
    {
        ROS_INFO("Got param: /ethercat_slaves/frequency = %d\n", FREQUENCY);
    }
    else
    {
        ROS_FATAL("Failed to get param '/ethercat_slaves/frequency'\n");
    }

    if (n.getParam("/ethercat_slaves/run_time", RUN_TIME))
    {
        ROS_INFO("Got param: /ethercat_slaves/run_time = %d\n", RUN_TIME);
    }
    else
    {
        ROS_FATAL("Failed to get param '/ethercat_slaves/run_time'\n");
    }
    PERIOD_NS = (NSEC_PER_SEC / FREQUENCY);

    ROS_INFO("Number of slaves in bus: %u", master_info.slave_count);
    ethercat_slaves = new slave_struct[master_info.slave_count];
    for (int i = 0; i < master_info.slave_count; i++)
    {
        ethercat_slaves[i].id = i;
        ethercat_slaves[i].slave_name = slave_names[i];
        ethercat_slaves[i].slave.init(ethercat_slaves[i].slave_name, n);
    }

    /******************************************
    *    Application domain data              *
    *******************************************/
   
    total_process_data = ecrt_domain_size(domain1);
    ROS_INFO("Number of total process data bytes: %lu\n", total_process_data);
    num_process_data_in = total_process_data - ethercat_slaves[master_info.slave_count - 1].slave.get_pdo_in();
    ROS_INFO("Number of process data input bytes for every slave: %lu\n", num_process_data_in);
    num_process_data_out = ethercat_slaves[master_info.slave_count - 1].slave.get_pdo_in() - ethercat_slaves[master_info.slave_count - 1].slave.get_pdo_out();
    ROS_INFO("Number of process data output bytes for every slave: %lu\n", num_process_data_out);
    process_data_buf = (uint8_t *)malloc(total_process_data * sizeof(uint8_t));
    memset(process_data_buf, 0, total_process_data); // fill the buffer with zeros
    n.setParam("/ethercat_slaves/slaves_count", (int)master_info.slave_count);

    //Initialize the Ethercat Communicator and the Ethercat Data Handlers
    ethercat_comm.init(n);
    ethercat_input_data_handler.init(n);
    ethercat_output_data_handler.init(n);

    /******************************************
    *    Launch the ROS services              *
    *******************************************/

    ros::ServiceServer modify_output_bit_service = n.advertiseService("modify_output_bit", modify_output_bit);
    ROS_INFO("Ready to modify output bit.");
    ros::ServiceServer modify_output_sbyte_service = n.advertiseService("modify_output_sbyte", modify_output_sbyte);
    ROS_INFO("Ready to modify output sbyte.");
    ros::ServiceServer modify_output_uint16_service = n.advertiseService("modify_output_uint16", modify_output_uint16);
    ROS_INFO("Ready to modify output uint16.");
    ros::ServiceServer modify_output_sint16_service = n.advertiseService("modify_output_sint16", modify_output_sint16);
    ROS_INFO("Ready to modify output sint16.");
    ros::ServiceServer modify_output_sint32_service = n.advertiseService("modify_output_sint32", modify_output_sint32);
    ROS_INFO("Ready to modify output sint32.");
    ros::ServiceServer ethercat_communicatord_service = n.advertiseService("ethercat_communicatord", ethercat_communicatord);
    ROS_INFO("Ready to communicate via EtherCAT.");

#ifdef MEASURE_TIMING
    /******************************************
    *           Open Log file                 *
    *******************************************/

    static char file_name[100];
    sprintf(file_name, "./outputs/measure_test_2/nort_ectest_dc_res_time_%d_%d_Hz_%d_s.csv", NUM_SLAVES, FREQUENCY, RUN_TIME);
    mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH;
    int flags = O_WRONLY | O_CREAT | O_TRUNC;
    log_fd = open(file_name, flags, mode);
    if (log_fd < 0)
    {
        ROS_FATAL("Could not open fd");
        exit(1);
    }
#endif

    ros::spin();
}

/*****************************************************************************/

/** @} */