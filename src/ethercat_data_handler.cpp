#include "ethercat_data_handler.h"
#include "ighm_ros/EthercatData.h"
#include "ighm_ros/EthercatRawData.h"
#include "ethercat_slave.h"
#include "utilities.h"
#include "vector"
#include "ighm_ros.h"
#include <iostream>
#include <string>

void EthercatDataHandler::raw_data_callback(const ighm_ros::EthercatRawData::ConstPtr &ethercat_data_raw)
{
    std::vector<uint8_t> input_data_raw = ethercat_data_raw->input_data_raw;
    uint8_t *data_ptr;
    size_t pos;
    for (int i = 0; i < master_info.slave_count; i++)
    {
        pos = ethercat_slaves[i].slave.get_pdo_in();
        data_ptr = (uint8_t * ) & input_data_raw[pos];
        ighm_ros::EthercatData ethercat_data;
        ethercat_data.hip_angle = process_input_sint16(data_ptr, 0);
        ethercat_data.desired_hip_angle = process_input_sint16(data_ptr, 2);
        ethercat_data.time = process_input_uint16(data_ptr, 4);
        ethercat_data.knee_angle = process_input_sint16(data_ptr, 6);
        ethercat_data.desired_knee_angle = process_input_sint16(data_ptr, 8);
        ethercat_data.PWM10000_knee = process_input_sint16(data_ptr, 10);
        ethercat_data.PWM10000_hip = process_input_sint16(data_ptr, 12);
        ethercat_data.velocity_knee1000 = process_input_sint32(data_ptr, 14);
        ethercat_data.velocity_hip1000 = process_input_sint32(data_ptr, 18);
        data_pub_[i].publish(ethercat_data);
    }
}

void EthercatDataHandler::init(ros::NodeHandle &n)
{
    //Create  ROS subscriber for the Ethercat RAW data
    data_raw_sub_ = n.subscribe("ethercat_data_raw", 1000, &EthercatDataHandler::raw_data_callback, &ethercat_data_handler);

    //Create  ROS publishers for the Ethercat formatted data
    data_pub_ = new ros::Publisher[master_info.slave_count];
    for (int i = 0; i < master_info.slave_count; i++)
    {
        data_pub_[i] = n.advertise<ighm_ros::EthercatData>("ethercat_data_slave_" + std::to_string(i), 1000);
    }
}
