#include "ethercat_communicator.h"
#include "ighm_ros.h"
#include "utilities.h"
#include "ethercat_slave.h"
#include <string.h>
#include "ighm_ros/EthercatRawData.h"

static unsigned int counter = 0;
// static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};
char new_string[100];

// static uint8_t BLUE_LED_index = 4;
// static uint8_t knee_angle_index = 6;
// static uint8_t hip_angle_index = 0;
// static uint8_t measurement_index = 64;
// static uint8_t state_machine_id = 0;
// static uint8_t state_machine_subid = 0;
// static uint8_t x_cntr_traj_id = 24;
// static uint8_t y_cntr_traj_id = 26;
// static uint8_t a_ellipse_id = 28;
// static uint8_t b_ellipse_id = 30;
// static uint8_t traj_freq_id = 32;
// static uint8_t phase_deg_id = 34;
// static uint8_t flatness_param_id = 36;

int EthercatCommunicator::cleanup_pop_arg_ = 0;
bool EthercatCommunicator::running_thread_ = false;
ros::Publisher EthercatCommunicator::data_raw_pub_;

/*****************************************************************************/

#if MEASURE_TIMING == 1

uint32_t period_ns, exec_ns = 0, latency_ns = 0,
                    latency_min_ns[RUN_TIME * SAMPLING_FREQ] = {0}, latency_max_ns[RUN_TIME * SAMPLING_FREQ] = {0},
                    period_min_ns[RUN_TIME * SAMPLING_FREQ] = {0}, period_max_ns[RUN_TIME * SAMPLING_FREQ] = {0},
                    exec_min_ns[RUN_TIME * SAMPLING_FREQ] = {0}, exec_max_ns[RUN_TIME * SAMPLING_FREQ] = {0};
#elif MEASURE_TIMING == 2
uint32_t latency_ns[RUN_TIME * FREQUENCY] = {0},
                               period_ns[RUN_TIME * FREQUENCY] = {0},
                               exec_ns[RUN_TIME * FREQUENCY] = {0};
#endif
/*****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC)
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    }
    else
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        ROS_INFO("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        ROS_INFO("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        ROS_INFO("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        ROS_INFO("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        ROS_INFO("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

bool EthercatCommunicator::has_running_thread()
{
    return running_thread_;
}

void EthercatCommunicator::init(ros::NodeHandle &n)
{

    const struct sched_param sched_param_ = {.sched_priority = 80};
    struct sched_param act_param = {};
    int act_policy;
    int ret;

    if (pthread_attr_init(&current_thattr_))
    {
        ROS_FATAL("Attribute init\n");
        exit(1);
    }
    if (pthread_attr_setdetachstate(&current_thattr_, PTHREAD_CREATE_JOINABLE))
    {
        ROS_FATAL("Attribute set detach state\n");
        exit(1);
    }
    if (pthread_attr_setinheritsched(&current_thattr_, PTHREAD_EXPLICIT_SCHED))
    {
        ROS_FATAL("Attribute set inherit schedule\n");
        exit(1);
    }
    if (pthread_attr_setschedpolicy(&current_thattr_, SCHED_FIFO))
    {
        ROS_FATAL("Attribute set schedule policy\n");
        exit(1);
    }
    // Get the values we just set, to make sure that they are set
    ret = pthread_attr_setschedparam(&current_thattr_, &sched_param_);
    if (ret != 0)
    {
        handle_error_en(ret, "pthread_attr_setschedparam");
    }
    ret = pthread_attr_getschedparam(&current_thattr_, &act_param);
    if (ret != 0)
    {
        handle_error_en(ret, "pthread_attr_getschedparam");
    }
    ret = pthread_attr_getschedpolicy(&current_thattr_, &act_policy);
    if (ret != 0)
    {
        handle_error_en(ret, "pthread_attr_getschedpolicy");
    }
    ROS_WARN("Actual pthread attribute values are: %d , %d\n", act_policy, act_param.sched_priority);

    //Create  ROS publisher for the Ethercat RAW data
    data_raw_pub_ = n.advertise<ighm_ros::EthercatRawData>("ethercat_data_raw", 1000);
}

void EthercatCommunicator::start()
{
    int ret;

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
    running_thread_ = true;

    memset(process_data_buf, 0, num_process_data); // fill the buffer with zeros
    
    ret = pthread_create(&communicator_thread_, &current_thattr_, &EthercatCommunicator::run, NULL);
    if (ret != 0)
    {
        handle_error_en(ret, "pthread_create");
    }
    ROS_INFO("Starting cyclic thread.\n");
}

void EthercatCommunicator::cleanup_handler(void *arg)
{
    ROS_INFO("Called clean-up handler\n");
}

void *EthercatCommunicator::run(void *arg)
{
    ros::Rate loop_rate(FREQUENCY);
    pthread_cleanup_push(EthercatCommunicator::cleanup_handler, NULL);
#ifdef MEASURE_TIMING
    struct timespec start_time, end_time, last_start_time = {};
#endif

    struct timespec break_time, current_time, offset_time = {RUN_TIME, 0}, wakeup_time;

    int ret;
    int i = 0;

    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeup_time);
    clock_gettime(CLOCK_TO_USE, &break_time);
    break_time = timespec_add(break_time, offset_time);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL); //PTHREAD_CANCEL_DEFERRED is the default but nevertheless

    do
    {
        pthread_testcancel();                                       // check if there is a request for cancel
        ret = pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL); //set the cancel state to DISABLE
        if (ret != 0)
        {
            handle_error_en(ret, "pthread_setcancelstate");
        }
        wakeup_time = timespec_add(wakeup_time, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeup_time, NULL);
#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &start_time);
#if MEASURE_TIMING == 1
        latency_ns = DIFF_NS(wakeup_time, start_time);
        period_ns = DIFF_NS(last_start_time, start_time);
        exec_ns = DIFF_NS(last_start_time, end_time);

        if (latency_ns > latency_max_ns[i])
        {
            latency_max_ns[i] = latency_ns;
        }
        if (latency_ns < latency_min_ns[i])
        {
            latency_min_ns[i] = latency_ns;
        }
        if (period_ns > period_max_ns[i])
        {
            period_max_ns[i] = period_ns;
        }
        if (period_ns < period_min_ns[i])
        {
            period_min_ns[i] = period_ns;
        }
        if (exec_ns > exec_max_ns[i])
        {
            exec_max_ns[i] = exec_ns;
        }
        if (exec_ns < exec_min_ns[i])
        {
            exec_min_ns[i] = exec_ns;
        }
#elif MEASURE_TIMING == 2
        latency_ns[i] = DIFF_NS(wakeup_time, start_time);
        period_ns[i] = DIFF_NS(last_start_time, start_time);
        exec_ns[i] = DIFF_NS(last_start_time, end_time);
#endif
        last_start_time = start_time;
#endif

        // receive process data
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        // for (int j = 0; j < NUM_SLAVES; j++)
        // {
        //     hip_angle[j] = process_input_sint16(domain1_pd + ethercat_slaves[j].slave.get_pdo_in(), hip_angle_index, hip_angle_index + 1);
        //     knee_angle[j] = process_input_sint16(domain1_pd + ethercat_slaves[j].slave.get_pdo_in(), knee_angle_index, knee_angle_index + 1);
        //     ROS_INFO("Angles: %d , %d \n", hip_angle[j], knee_angle[j]);
        // }
        // printf("I:");
        // for (size_t k = ethercat_slaves[0].slave.get_pdo_in(); k < num_process_data; k++)
        //     printf(" %2.2x", *(domain1_pd + k));
        // printf("\n");
        // check process data state (optional)
        check_domain1_state();

        if (counter)
        {
            counter--;
        }
        else
        { // do this at 10 Hz
#if MEASURE_TIMING == 1
            counter = FREQUENCY / SAMPLING_FREQ;
#elif MEASURE_TIMING == 2
            counter = 0;
#endif
            i++;
            // check for master state (optional)
            check_master_state();

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
            // blink = !blink;

            // calculate new process data
        }

        // write process data
        // modify_output_bit(domain1_pd + pdo_out, 0, BLUE_LED_index, blink);
        // modify_output_bit(domain1_pd+pdo_out, measurement_index,1);
        // modify_output_bit(domain1_pd+pdo_out, measurement_index,1);
        // modify_output_sint16(domain1_pd + pdo_out, x_cntr_traj_id, 0);
        // modify_output_sint16(domain1_pd + pdo_out, y_cntr_traj_id, 590);
        // modify_output_sint16(domain1_pd + pdo_out, a_ellipse_id, 0);
        // modify_output_sint16(domain1_pd + pdo_out, b_ellipse_id, 3);
        // modify_output_sint16(domain1_pd + pdo_out, traj_freq_id, 100);
        // modify_output_sint16(domain1_pd + pdo_out, phase_deg_id, 0);
        // modify_output_sint16(domain1_pd + pdo_out, flatness_param_id, 0);
        // modify_output_bit(domain1_pd + pdo_out, state_machine_id, state_machine_subid, 1);

        // write application time to master
        clock_gettime(CLOCK_TO_USE, &current_time);
        ecrt_master_application_time(master, TIMESPEC2NS(current_time));

        if (sync_ref_counter)
        {
            sync_ref_counter--;
        }
        else
        {
            sync_ref_counter = 5; // sync every 5 cycles
            ecrt_master_sync_reference_clock(master);
        }
        ecrt_master_sync_slave_clocks(master);

        //move the data from process_data_buf to domain1_pd buf carefuly
        EthercatCommunicator::copy_data_to_domain_buf();

        //send the raw data to the raw data topic
        EthercatCommunicator::publish_raw_data();

        // send process data
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        int ret = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL); //set the cancel state to ENABLE
        if (ret != 0)
        {
            handle_error_en(ret, "pthread_setcancelstate");
        }
#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &end_time);
#endif
        clock_gettime(CLOCK_TO_USE, &current_time);
    } while (DIFF_NS(current_time, break_time) > 0);

    // write the statistics to file
#if MEASURE_TIMING == 1
    for (i = 0; i < RUN_TIME * SAMPLING_FREQ; i++)
    {
        snprintf(new_string, 100, "%10u , %10u , 10u , %10u , 10u , %10u\n",
                 period_min_ns[i], period_max_ns[i], exec_min_ns[i], exec_max_ns[i],
                 latency_min_ns[i], latency_max_ns[i]);
        fprintf(file, "%s", string_file);
    }
#elif MEASURE_TIMING == 2
    for (i = 0; i < RUN_TIME * FREQUENCY; i++)
    {
        if (i % 10000 == 0)
            ROS_INFO("Current line written is: %d\n", i);
        snprintf(new_string, sizeof(new_string), "%10u , %10u , %10u\n",
                 period_ns[i], exec_ns[i], latency_ns[i]);
        if ((uint32_t)insist_write(log_fd, new_string, strlen(new_string)) != strlen(new_string))
        {
            ROS_FATAL("ec_thread: insist_write");
            exit(1);
        }
    }
#endif
#ifdef MEASURE_TIMING
    if (close(log_fd))
    {
        ROS_ERROR("ec_thread: close log fd");
        exit(1);
    }
#endif
    pthread_cleanup_pop(cleanup_pop_arg_);
    running_thread_ = false;
    exit(0);
}

void EthercatCommunicator::stop()
{

    int ret;
    void *res;

    ROS_INFO("stop(): sending cancellation request\n");
    ret = pthread_cancel(communicator_thread_);
    if (ret != 0)
        handle_error_en(ret, "pthread_cancel");

    /* Join with thread to see what its exit status was */

    ret = pthread_join(communicator_thread_, &res);
    if (ret != 0)
        handle_error_en(ret, "pthread_join");

    if (res == PTHREAD_CANCELED)
    {
        ROS_INFO("stop(): communicator thread  was canceled\n");
        running_thread_ = false;
    }
    else
        ROS_INFO("stop(): communicator thread wasn't canceled (shouldn't happen!)\n");
}

void EthercatCommunicator::copy_data_to_domain_buf()
{
    pthread_spin_lock(&lock);
    for (int i = 0; i < master_info.slave_count; i++)
    {
        memcpy((domain1_pd + ethercat_slaves[i].slave.get_pdo_out()),
               (process_data_buf + ethercat_slaves[i].slave.get_pdo_out()),
               (size_t)(ethercat_slaves[i].slave.get_pdo_in() - ethercat_slaves[i].slave.get_pdo_out()));
    }
    pthread_spin_unlock(&lock);
}

void EthercatCommunicator::publish_raw_data()
{
    std::vector<uint8_t> input_data_raw, output_data_raw;
    //Create input data raw string
    std::vector<uint8_t> input_vec, output_vec;
    unsigned char * raw_data_pointer;
    for (int i = 0; i < master_info.slave_count; i++)
    {
        raw_data_pointer = (unsigned char *)domain1_pd + ethercat_slaves[i].slave.get_pdo_in();
        input_vec.insert(std::end(input_vec), raw_data_pointer, raw_data_pointer + num_process_data_in);
    }
    input_data_raw.insert(std::end(input_data_raw), std::begin(input_vec), std::end(input_vec));
    //Create output data raw string
    for (int i = 0; i < master_info.slave_count; i++)
    {
        raw_data_pointer = (unsigned char *)domain1_pd + ethercat_slaves[i].slave.get_pdo_out();
        output_vec.insert(std::end(output_vec), raw_data_pointer, raw_data_pointer + num_process_data_out);
    }
    output_data_raw.insert(std::end(output_data_raw), std::begin(output_vec), std::end(output_vec));
    //Send both strings to the topic
    ighm_ros::EthercatRawData raw_data;
    raw_data.input_data_raw = input_data_raw;
    raw_data.output_data_raw = output_data_raw;
    data_raw_pub_.publish(raw_data);
}
