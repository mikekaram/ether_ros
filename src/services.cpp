
#include "services.h"
#include "ecrt.h"
#include "ethercat_communicator.h"
#include "ighm_ros.h"

EthercatCommunicator ec;

bool modify_output_bit(ighm_ros::ModifyOutputBit::Request &req,
                       ighm_ros::ModifyOutputBit::Response &res)
{
    uint8_t *data_ptr = process_data_buf;
    uint8_t *new_data_ptr = (data_ptr + req.index);
    pthread_spin_lock(&lock);
    EC_WRITE_BIT(new_data_ptr, req.subindex, req.value);
    pthread_spin_unlock(&lock);
    res.success = "true";
    return true;
}

bool modify_output_uint16(ighm_ros::ModifyOutputUInt16::Request &req,
                          ighm_ros::ModifyOutputUInt16::Response &res)
{
    uint8_t *data_ptr = process_data_buf;
    uint8_t *new_data_ptr = (data_ptr + req.index);
    pthread_spin_lock(&lock);
    EC_WRITE_U16((uint16_t *)(data_ptr + req.index), req.value);
    pthread_spin_unlock(&lock);
    res.success = "true";
    return true;
}

bool modify_output_sint16(ighm_ros::ModifyOutputSInt16::Request &req,
                          ighm_ros::ModifyOutputSInt16::Response &res)
{
    uint8_t *data_ptr = process_data_buf;
    uint8_t *new_data_ptr = (data_ptr + req.index);
    pthread_spin_lock(&lock);
    EC_WRITE_S16((int16_t *)(data_ptr + req.index), req.value);
    pthread_spin_unlock(&lock);
    res.success = "true";
    return true;
}

bool modify_output_sint32(ighm_ros::ModifyOutputSInt32::Request &req,
                          ighm_ros::ModifyOutputSInt32::Response &res)
{
    uint8_t *data_ptr = process_data_buf;
    uint8_t *new_data_ptr = (data_ptr + req.index);
    pthread_spin_lock(&lock);
    EC_WRITE_S32((int32_t *)(data_ptr + req.index), req.value);
    pthread_spin_unlock(&lock);
    res.success = "true";
    return true;
}
bool ethercat_communicatord(ighm_ros::EthercatCommd::Request &req,
                            ighm_ros::EthercatCommd::Response &res)
{
    bool s;
    if (req.mode != "start")
    {
        s = start_ethercat_communicator();
        res.success = s ? "true" : "false";
        return s;
    }
    else if (req.mode != "stop")
    {
        s = stop_ethercat_communicator();
        res.success = s ? "true" : "false";
        return s;
    }
    else
        return false;
}

bool start_ethercat_communicator()
{
    if (!(bool)ec.has_running_thread())
    {
        ec.init();
        ec.start();
        return true;
    }
    else
        return false;
}

bool stop_ethercat_communicator()
{
    if ((bool)ec.has_running_thread())
    {
        ec.stop();
        return true;
    }
    else
        return false;
}
