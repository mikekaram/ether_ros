#include "utilities.h"
#include "services.h"
#include "ighm_ros.h"
#include "ethercat_communicator.h"

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
EthercatInputDataHandler ethercat_input_data_handler;
EthercatOutputDataHandler ethercat_output_data_handler;
int FREQUENCY;
int RUN_TIME;
int PERIOD_NS;
/****************************************************************************/
// EtherCAT
// extern ec_master_t *master = NULL;
// static ec_master_state_t master_state = {};

// static ec_domain_t *domain1 = NULL;
// static ec_domain_state_t domain1_state = {};

/****************************************************************************/

// process data

// #define ForeLeg    0, 0

// #define BECKHOFF_FB1111 0x00000A12, 0x00A986FD
// #define XMC_4800 0x00000A12, 0x00000000
// #define IDS_Counter     0x000012ad, 0x05de3052

// offsets for PDO entries

/****************************************************************************/

static char file_name[100];

int main(int argc, char **argv)
{
    int ret;

    std::stringstream ss;
    std::string s;
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
