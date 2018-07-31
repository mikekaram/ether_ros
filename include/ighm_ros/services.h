#ifndef SERV_LIB_H
#define SERV_LIB_H

#include "ros/ros.h"
#include "ighm_ros/ModifyOutputBit.h"
#include "ighm_ros/ModifyOutputUInt16.h"
#include "ighm_ros/ModifyOutputSInt16.h"
#include "ighm_ros/ModifyOutputSInt32.h"
#include "ighm_ros/EthercatCommd.h"

bool modify_output_bit(ighm_ros::ModifyOutputBit::Request &req,
                       ighm_ros::ModifyOutputBit::Response &res);

bool modify_output_sbyte(ighm_ros::ModifyOutputBit::Request &req,
                       ighm_ros::ModifyOutputBit::Response &res);

bool modify_output_uint16(ighm_ros::ModifyOutputUInt16::Request &req,
                          ighm_ros::ModifyOutputUInt16::Response &res);

bool modify_output_sint16(ighm_ros::ModifyOutputSInt16::Request &req,
                          ighm_ros::ModifyOutputSInt16::Response &res);

bool modify_output_sint32(ighm_ros::ModifyOutputSInt32::Request &req,
                          ighm_ros::ModifyOutputSInt32::Response &res);

bool ethercat_communicatord(ighm_ros::EthercatCommd::Request &req,
                            ighm_ros::EthercatCommd::Response &res);

bool start_ethercat_communicator();
bool stop_ethercat_communicator();

#endif /* SERV_LIB_H */
