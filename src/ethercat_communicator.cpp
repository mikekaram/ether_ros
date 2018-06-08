#include "ethercat_communicator.h"
#include "ighm_ros.h"

static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};
char new_string[100];
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


void EthercatCommunicator::EthercatCommunicator(){

    communicator_thread_ = NULL;
}

std::bool has_running_thread::EthercatCommunicator(){

    if (communicator_thread_ == NULL)
        return false;
    else return true;
}
void init::EthercatCommunicator(){

    const struct sched_param sched_param_ = {.sched_priority = 80};
    struct sched_param act_param = {};
    int act_policy;
    int ret;

    if (pthread_attr_init(&current_thattr_)){
        ROS_FATAL("Attribute init\n");
        exit(1);
    }
    if (pthread_attr_setdetachstate(&current_thattr_, PTHREAD_CREATE_JOINABLE)){
        ROS_FATAL("Attribute set detach state\n");
        exit(1);
    }
    if (pthread_attr_setinheritsched(&current_thattr_, PTHREAD_EXPLICIT_SCHED)){
        ROS_FATAL("Attribute set inherit schedule\n");
        exit(1);
    }
    if (pthread_attr_setschedpolicy(&current_thattr_, SCHED_FIFO)){
        ROS_FATAL("Attribute set schedule policy\n");
        exit(1);
    }
    // Get the values we just set, to make sure that they are set
    ret = pthread_attr_setschedparam(&current_thattr_, &sched_param_);
    if (ret != 0){
        handle_error_en(ret, "pthread_attr_setschedparam");
    }
    ret = pthread_attr_getschedparam(&current_thattr_, &act_param);
    if (ret != 0){
        handle_error_en(ret, "pthread_attr_getschedparam");
    }
    ret = pthread_attr_getschedpolicy(&current_thattr_, &act_policy);
    if (ret != 0){
        handle_error_en(ret, "pthread_attr_getschedpolicy");
    }
    ROS_WARN("Actual pthread attribute values are: %d , %d\n", act_policy, act_param.sched_priority);
}

void start::EthercatCommunicator(){
    int ret;
    ret = pthread_create(communicator_thread_, &current_thattr_, &EthercatCommunicator.run(), NULL);
    if (ret != 0){
        handle_error_en(ret, "pthread_create");
    }
    ROS_INFO("Starting cyclic thread.\n");
}
static void cleanup_handler::EthercatCommunicator(void *arg) {
    ROS_INFO("Called clean-up handler\n");
}
void run::EthercatCommunicator(void arg*){

    pthread_cleanup_push(EthercatCommunicator.cleanup_handler, NULL);
    #ifdef MEASURE_TIMING
        struct timespec start_time, end_time, last_start_time = {};
    #endif

    struct timespec break_time, current_time, offset_time = {RUN_TIME, 0}, wakeup_time;
    uint16_t noise;
    uint8_t pdo_in_end = 5;
    int32_t hip_angle, knee_angle;

    int ret;
    int i = 0;

    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeup_time);
    clock_gettime(CLOCK_TO_USE, &break_time);
    break_time = timespec_add(break_time, offset_time);
    do{
        pthread_testcancel(); // check if there is a request for cancel
        ret = pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &cancel_state); //set the cancel state to DISABLE
        if (ret != 0) {
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
        #elif MEASURE_TIMING == 2
        latency_ns[i] = DIFF_NS(wakeup_time, start_time);
        period_ns[i] = DIFF_NS(last_start_time, start_time);
        exec_ns[i] = DIFF_NS(last_start_time, end_time);
    #endif
        last_start_time = start_time;

    #if MEASURE_TIMING == 1
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
    #endif
        // receive process data
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        // noise = process_input_uint16(domain1_pd + off_counter_in,0);

        hip_angle = process_input_sint16(domain1_pd + pdo_in, hip_angle_index, hip_angle_index + 1);
        knee_angle = process_input_sint16(domain1_pd + pdo_in, knee_angle_index, knee_angle_index + 1);
        printf("Angles: %d , %d \n", hip_angle, knee_angle);
        printf("I:");
        for (int j = pdo_in; j < 60; j++)
            printf(" %2.2x", *(domain1_pd + j));
        printf("\n");
        // check process data state (optional)
        // check_domain_state();

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
        modify_output_bit(domain1_pd + pdo_out, 0, BLUE_LED_index, blink);
        // modify_output_bit(domain1_pd+pdo_out, measurement_index,1);
        // modify_output_bit(domain1_pd+pdo_out, measurement_index,1);
        modify_output_sint16(domain1_pd + pdo_out, x_cntr_traj_id, 0);
        modify_output_sint16(domain1_pd + pdo_out, y_cntr_traj_id, 590);
        modify_output_sint16(domain1_pd + pdo_out, a_ellipse_id, 0);
        modify_output_sint16(domain1_pd + pdo_out, b_ellipse_id, 3);
        modify_output_sint16(domain1_pd + pdo_out, traj_freq_id, 100);
        modify_output_sint16(domain1_pd + pdo_out, phase_deg_id, 0);
        modify_output_sint16(domain1_pd + pdo_out, flatness_param_id, 0);
        modify_output_bit(domain1_pd + pdo_out, state_machine_id, state_machine_subid, 1);
        printf("O:");
        for (int j = pdo_out; j < 38; j++)
            printf(" %2.2x", *(domain1_pd + j));
        printf("\n");

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

        // send process data
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        int ret = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &cancel_state); //set the cancel state to ENABLE
        if (ret != 0){
            handle_error_en(ret, "pthread_setcancelstate");
        }
#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &end_time);
    #endif
        clock_gettime(CLOCK_TO_USE, &current_time);
    }
    while (DIFF_NS(current_time, break_time) > 0);

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
    pthread_cleanup_pop(cleanup_pop_arg);
    exit(0);
}


void stop::EthercatCommunicator(){

    int ret;
    void *res;

    ROS_INFO("stop(): sending cancellation request\n");
    ret = pthread_cancel(*communicator_thread_);
    if (ret != 0)
        handle_error_en(ret, "pthread_cancel");

    /* Join with thread to see what its exit status was */

    ret = pthread_join(*communicator_thread_, &res);
    if (ret != 0)
        handle_error_en(ret, "pthread_join");

    if (res == PTHREAD_CANCELED)
        ROS_INFO("stop(): communicator thread  was canceled\n");
    else
        ROS_INFO("stop(): communicator thread wasn't canceled (shouldn't happen!)\n");
}

