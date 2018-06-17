
#include "services.h"
#include "ecrt.h"
#include "ethercat_communicator.h"
#include "ighm_ros.h"

EthercatCommunicator ec;

//See more at: http://www.martinbroadhurst.com/how-to-trim-a-stdstring.html
// trim string from start
std::string& ltrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
    str.erase(0, str.find_first_not_of(chars));
    return str;
}
 
std::string& rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
    str.erase(str.find_last_not_of(chars) + 1);
    return str;
}
 
std::string& trim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
{
    return ltrim(rtrim(str, chars), chars);
}

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
    req.mode = trim(req.mode);

    if (req.mode == "start")
    {
        s = start_ethercat_communicator();
        res.success = s ? "true" : "false";
        return true;
    }
    else if (req.mode == "stop")
    {
        s = stop_ethercat_communicator();

        res.success = s ? "true" : "false";
        return true;
    }
    else if (req.mode == "thread")
    {

        res.success = ec.has_running_thread() ? "true" : "false";
        return true;
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
