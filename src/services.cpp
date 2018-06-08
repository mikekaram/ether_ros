
#include "services.h"
#include "ecrt.h"
#include "ethercat_communicator.h"
#include "ighm_ros.h"

EthercatCommunicator ec;

std::string modify_output_bit(ighm_ros::ModifyOutputBit::Request &req,
                              ighm_ros::ModifyOutputBit::Response &res)
{
    uint8_t *new_data_ptr = &data_ptr[index];
    EC_WRITE_BIT(new_data_ptr, req.subindex, req.value);
    res.success = "true";
    return res.success;
}

std::string modify_output_uint16(ighm_ros::ModifyOutputUInt16::Request &req,
                                 ighm_ros::ModifyOutputUInt16::Response &res)
{
    uint8_t *new_data_ptr = &data_ptr[index];
    EC_WRITE_U16((uint16_t *)(data_ptr + req.index), req.value);
    res.success = "true";
    return res.success;
}

std::string modify_output_sint16(ighm_ros::ModifyOutputSInt16::Request &req,
                                 ighm_ros::ModifyOutputSInt16::Response &res)
{
    uint8_t *new_data_ptr = &data_ptr[index];
    EC_WRITE_S16((int16_t *)(data_ptr + req.index), req.value);
    res.success = "true";
    return res.success;
}

std::string modify_output_sint32(ighm_ros::ModifyOutputSInt32::Request &req,
                                 ighm_ros::ModifyOutputSInt32::Response &res)
{
    uint8_t *new_data_ptr = &data_ptr[index];
    EC_WRITE_S32((int32_t *)(data_ptr + req.index), req.value);
    res.success = "true";
    return res.success;
}
std::string ethercat_communicatord(ighm_ros::EthercatCommd::Request &req,
                                   ighm_ros::EthercatCommd::Response &res)
{
    std::string success;
    if(!req.mode == "start"){
        sucess = start_ethercat_communicator();
        return sucess;
    }
    else if(!req.mode == "stop"){
        sucess = stop_ethercat_communicator();
        return sucess;
    }
    else
        return "false";
}

std::string start_ethercat_communicator()
{
    if(!ec.has_running_thread){
        ec.init();
        ec.start();
        return "true";
    }
    else return "false";
}

std::string stop_ethercat_communicator()
{
    if(ec.has_running_thread){
        ec.stop();
        return "true";
    }
    else return "false";
}
