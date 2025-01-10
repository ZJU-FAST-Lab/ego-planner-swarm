#ifndef __QUADROTOR_MSGS_QUADROTOR_MSGS_H__
#define __QUADROTOR_MSGS_QUADROTOR_MSGS_H__

#include <stdint.h>
#include <cstring>
#include <vector>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <quadrotor_msgs/msg/trpy_command.hpp>
#include <quadrotor_msgs/msg/gains.hpp>

namespace quadrotor_msgs
{

void encodeSO3Command(const quadrotor_msgs::msg::SO3Command &so3_command,
                      std::vector<uint8_t> &output);
void encodeTRPYCommand(const quadrotor_msgs::msg::TRPYCommand &trpy_command,
                       std::vector<uint8_t> &output);

void encodePPRGains(const quadrotor_msgs::msg::Gains &gains,
                    std::vector<uint8_t> &output);
}

#endif
