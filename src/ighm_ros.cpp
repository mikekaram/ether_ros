

#include "utilities.h"
#include "services.h"
#include "ighm_ros.h"

// static uint8_t BLUE_LED_index = 4;
// static uint8_t knee_angle_index = 6;
// static uint8_t hip_angle_index = 0;
// static uint8_t measurement_index = 64;
// static  uint8_t state_machine_id = 0;
// static  uint8_t state_machine_subid = 0;
// static  uint8_t x_cntr_traj_id = 24;
// static  uint8_t y_cntr_traj_id = 26;
// static  uint8_t a_ellipse_id = 28;
// static  uint8_t b_ellipse_id = 30;
// static  uint8_t traj_freq_id = 32;
// static  uint8_t phase_deg_id = 34;
// static  uint8_t flatness_param_id = 36;

int run = 1;

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
    ec_slave_config_t *sc;

    int ret;

    std::stringstream ss;
    std::string s;
    int vendor_id, product_code, assign_activate, position, alias;

    ros::init(argc, argv, "ighm_ros");

    ros::NodeHandle n;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		ROS_FATAL("mlockall failed");
		exit(1);
	}

    master = ecrt_request_master(0);
    if (!master){
        ROS_FATAL("Failed to get master.\n");
        exit(1);
    }
    domain1 = ecrt_master_create_domain(master);
    if (!domain1){
        ROS_FATAL("Failed to create domain.\n");
        exit(1);
    }
    while(!n.getParam("/ethercat_slaves/vendor_id",vendor_id)){
        ROS_INFO("Waiting the parameter server to initialize\n");
    }
    ROS_INFO("Got param: /ethercat_slaves/vendor_id = %2.2x\n", vendor_id);

    if (n.getParam("/ethercat_slaves/fore_leg/alias", alias))
    {
        ROS_INFO("Got param: /ethercat_slaves/fore_leg/alias = %d\n", alias);
    }
    else {
        ROS_FATAL("Failed to get param '/ethercat_slaves/fore_leg/alias'\n");
    }

    if (n.getParam("/ethercat_slaves/fore_leg/position", position))
    {
        ROS_INFO("Got param: /ethercat_slaves/fore_leg/position = %d\n", position);
    }
    else {
      ROS_FATAL("Failed to get param 'ethercat_slaves/fore_leg/position'\n");
    }

    if (n.getParam("/ethercat_slaves/fore_leg/product_code", product_code)) {
        ROS_INFO("Got param: /ethercat_slaves/fore_leg/product_code = %2.2x\n", product_code);
    }
    else {
      ROS_FATAL("Failed to get param '/ethercat_slaves/fore_leg/product_code'\n");
    }

    if (n.getParam("/ethercat_slaves/fore_leg/assign_activate", assign_activate)) {
        ROS_INFO("Got param: /ethercat_slaves/fore_leg/assign_activate = %2.2x\n", assign_activate);
    }
    else {
      ROS_FATAL("Failed to get param '/ethercat_slaves/fore_leg/assign_activate'\n");
    }
    // Create configuration for bus coupler
    sc = ecrt_master_slave_config(master, alias, position, vendor_id, product_code);
    if (!sc){
    	ROS_FATAL("Failed to get slave configuration.\n");
        exit(1);
    }
    /************************************************

    *************************************************/

    ros::ServiceServer service = n.advertiseService("modify_output_bit", modify_output_bit);
    ROS_INFO("Ready to modify output bit.");
    ros::ServiceServer service = n.advertiseService("modify_output_uint16", modify_output_uint16);
    ROS_INFO("Ready to modify output uint16.");
    ros::ServiceServer service = n.advertiseService("modify_output_sint16", modify_output_sint16);
    ROS_INFO("Ready to modify output sint16.");
    ros::ServiceServer service = n.advertiseService("modify_output_sint32", modify_output_sint32);
    ROS_INFO("Ready to modify output sint32.");
    ros::ServiceServer service = n.advertiseService("ethercat_communicatord", ethercat_communicatord);
    ROS_INFO("Ready to communicate via EtherCAT.");


    pdo_out = ecrt_slave_config_reg_pdo_entry(sc,
            0x7000, 1, domain1, NULL);
    if (pdo_out < 0){
        ROS_FATAL("Failed to configure pdo out.\n");
        exit(1);
    }
    ROS_INFO("Offset pdo out is: %d\n", pdo_out);

	pdo_in = ecrt_slave_config_reg_pdo_entry(sc,
			0x6010, 1, domain1, NULL);
	if (pdo_in < 0){
        ROS_FATAL("Failed to configure pdo in.\n");
        exit(1);
    }
    ROS_INFO("Offset pdo in is: %d\n", pdo_in);

    // configure SYNC signals for this slave
    // shift time = 4400000
    //For XMC use: 0x0300
    //For Beckhoff FB1111 use: 0x0700
	// ecrt_slave_config_dc(sc, 0x0700, PERIOD_NS, PERIOD_NS/10, 0, 0);
    ecrt_slave_config_dc(sc, assign_activate, PERIOD_NS, PERIOD_NS/100, 0, 0);
    ROS_INFO("Activating master...\n");
    if (ecrt_master_activate(master)){
        ROS_FATAL("Failed to activate master.\n");
        exit(1);
    }
    domain1_pd = NULL;
    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        ROS_FATAL("Failed to set domain data.\n");
        exit(1);
    }

// ************************************************
    /* Open log file */
#ifdef MEASURE_TIMING
    sprintf(file_name,"./outputs/measure_test_2/nort_ectest_dc_res_time_%d_%d_Hz_%d_s.csv",NUM_SLAVES,FREQUENCY,RUN_TIME);
    mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH;
    int flags = O_WRONLY | O_CREAT | O_TRUNC;
    log_fd = open(file_name, flags, mode);
    if(log_fd < 0){
        ROS_FATAL("Could not open fd");
        exit(1);
    }
#endif
    ros::spin();
}



