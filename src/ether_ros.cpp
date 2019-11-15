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
 *  Public License as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace program in the ROS environment. If not, see
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

//\defgroup EtherROS EtherCAT module main

/** \file ether_ros.cpp
 *
 * \brief Main source file.
 *
 * IgH Master EtherCAT module main for realtime communication with EtherCAT slaves.
 * This interface is designed for realtime modules, running in the ROS environment
 * and want to use EtherCAT. There are classes and functions to get the Input and Output PDOs
 * in a realtime context. Before trying to understand the source code of the project,
 * please consider to read and understand the IgH Master Documentation located at:
 * https://www.etherlab.org/download/ethercat/ethercat-1.5.2.pdf. Finally try to
 * understand the API provided in the /opt/etherlab/include/ecrt.h file. The author admits that
 * the C++ language, is not his strong suit. Therefore feel free to refactore the code given with
 * use of the new C++ (11/14/17) helpful tools (unique/shared pointers and other cool stuff).
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
 *   Service API for Input PDO handling and topic streaming: /ethercat_data_slave_{slave_id}
 *
 * - Added synchronization primitives (spinlocks) for the concurrent threads accessing the EtherCAT buffer.
<<<<<<< HEAD:src/ighm_ros.cpp
 * - Added more source files, ethercat communicator, ethercat_slave, ethercat_input_data_handler and
 * ethercat_output_data_handler. Created external objects to use the appropriate classes and functions.
=======
 * - Added more source files, ethercat communicator, ethercat_slave, pdo_in_publisher and
 * pdo_out_publisher. Created external objects to use the appropriate classes and functions.
>>>>>>> devel:src/ether_ros.cpp
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
#include "ether_ros.h"

/*****************************************************************************/

/************************GLOBAL VARIABLES ************************************/

uint8_t *domain1_pd;
uint8_t *process_data_buf;
size_t total_process_data;
size_t num_process_data_in;
size_t num_process_data_out;
int log_fd;
ec_master_t *master;
ec_master_state_t master_state;
ec_master_info_t master_info;
ec_domain_t *domain1;
ec_domain_state_t domain1_state;
slave_struct *ethercat_slaves;
pthread_spinlock_t lock;
EthercatCommunicator ethercat_comm;
<<<<<<< HEAD:src/ighm_ros.cpp
EthercatInputDataHandler ethercat_input_data_handler;
EthercatOutputDataHandler ethercat_output_data_handler;
=======
PDOInPublisher pdo_in_publisher;
PDOOutPublisher pdo_out_publisher;
PDOOutListener pdo_out_listener;
PDOOutPublisherTimer pdo_out_publisher_timer;
>>>>>>> devel:src/ether_ros.cpp
int FREQUENCY;
int RUN_TIME;
int PERIOD_NS;

/****************************************************************************/

int main(int argc, char **argv)
{
    int ret;
    std::string slave_names[4] = {"front_left_leg", "front_right_leg", "back_right_leg", "back_left_leg"};

    ros::init(argc, argv, "ether_ros");

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
    if (n.getParam("/ethercat_slaves/period_ns", PERIOD_NS))
    {
        ROS_INFO("Got param: /ethercat_slaves/period_ns = %d\n", PERIOD_NS);
    }
    else
    {
        ROS_FATAL("Failed to get param '/ethercat_slaves/period_ns'\n");
    }

    if (n.getParam("/ethercat_slaves/run_time", RUN_TIME))
    {
        ROS_INFO("Got param: /ethercat_slaves/run_time = %d\n", RUN_TIME);
    }
    else
    {
        ROS_FATAL("Failed to get param '/ethercat_slaves/run_time'\n");
    }
    FREQUENCY = (NSEC_PER_SEC / PERIOD_NS);

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

    n.setParam("/ethercat_slaves/slaves_count", (int)master_info.slave_count); // set the slaves_count to the actual slaves found and configured

    //Initialize the Ethercat Communicator and the Ethercat Data Handlers
    ethercat_comm.init(n);
    pdo_in_publisher.init(n);
    // pdo_out_publisher.init(n);
    pdo_out_listener.init(n);
    pdo_out_publisher_timer.init(n);


    /******************************************
    *    Launch the ROS services              *
    *******************************************/
    ros::ServiceServer ethercat_communicatord_service = n.advertiseService("ethercat_communicatord", ethercat_communicatord);
    ROS_INFO("Ready to communicate via EtherCAT.");

#ifdef LOGGING
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
