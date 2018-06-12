#include "utilities.h"
#include "services.h"
#include "ighm_ros.h"

uint8_t *domain1_pd;
int log_fd;
ec_master_t *master;
ec_master_state_t master_state;
ec_domain_t *domain1;
ec_domain_state_t domain1_state;
ros::NodeHandle n;
slave_struct ethercat_slaves[NUM_SLAVES];
pthread_spinlock_t *lock;
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

static char file_name[100];

/****************************************************************************/

int main(int argc, char **argv)
{
    int ret;

    std::stringstream ss;
    std::string s;
    std::string slave_names[NUM_SLAVES] = {"front_left_leg"}; //"front_right_leg" "back_right_leg" "back_left_leg"

    ros::init(argc, argv, "ighm_ros");

    ros::NodeHandle n;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        ROS_FATAL("mlockall failed");
        exit(1);
    }

    ret = pthread_spin_init(lock, PTHREAD_PROCESS_SHARED); // It really doesn't matter for our application what pshared value we use
    if (!ret)
    {
        handle_error_en(ret, "pthread_spin_lock_init");
    }
    master = ecrt_request_master(0);
    if (!master)
    {
        ROS_FATAL("Failed to get master.\n");
        exit(1);
    }
    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
        ROS_FATAL("Failed to create domain.\n");
        exit(1);
    }
    for (int i = 0; i < NUM_SLAVES; i++)
    {
        ethercat_slaves[i].id = i;
        ethercat_slaves[i].slave_name = slave_names[i];
        ethercat_slaves[i].slave.initialize(ethercat_slaves[i].slave_name);
    }
    // Create configuration for bus coupler

    /************************************************
        Launch the ROS services
    *************************************************/

    ros::ServiceServer service = n.advertiseService("modify_output_bit", modify_output_bit);
    ROS_INFO("Ready to modify output bit.");
    service = n.advertiseService("modify_output_uint16", modify_output_uint16);
    ROS_INFO("Ready to modify output uint16.");
    service = n.advertiseService("modify_output_sint16", modify_output_sint16);
    ROS_INFO("Ready to modify output sint16.");
    service = n.advertiseService("modify_output_sint32", modify_output_sint32);
    ROS_INFO("Ready to modify output sint32.");
    service = n.advertiseService("ethercat_communicatord", ethercat_communicatord);
    ROS_INFO("Ready to communicate via EtherCAT.");

    ROS_INFO("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        ROS_FATAL("Failed to activate master.\n");
        exit(1);
    }
    domain1_pd = NULL;
    if (!(domain1_pd = ecrt_domain_data(domain1)))
    {
        ROS_FATAL("Failed to set domain data.\n");
        exit(1);
    }

    // ************************************************
    /* Open log file */
#ifdef MEASURE_TIMING
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
