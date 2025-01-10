#ifndef __QUADROTOR_MSGS_QUADROTOR_MSGS_H__
#define __QUADROTOR_MSGS_QUADROTOR_MSGS_H__

#include <stdint.h>
#include <cstring> 
#include <vector>
#include <quadrotor_msgs/msg/output_data.hpp>
#include <quadrotor_msgs/msg/status_data.hpp>
#include <quadrotor_msgs/msg/ppr_output_data.hpp>

namespace quadrotor_msgs
{

bool decodeOutputData(const std::vector<uint8_t> &data,
                      quadrotor_msgs::msg::OutputData &output);

bool decodeStatusData(const std::vector<uint8_t> &data,
                      quadrotor_msgs::msg::StatusData &status);

bool decodePPROutputData(const std::vector<uint8_t> &data,
                         quadrotor_msgs::msg::PPROutputData &output);
}

#endif
