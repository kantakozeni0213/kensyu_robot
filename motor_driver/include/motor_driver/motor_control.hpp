#ifndef VELOCITY_NODE_HPP
#define VELOCITY_NODE_HPP

#include <cstdio>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>

// Control table address (Dynamixel X-series)
#define ADDR_OPERATING_MODE                    11
#define ADDR_TORQUE_ENABLE                     64
#define ADDR_GOAL_CURRENT                      102	 // Does NOT exist in Rot motors
#define ADDR_GOAL_VELOCITY                     104
#define ADDR_GOAL_POSITION                     116
#define ADDR_PRESENT_CURRENT                   126	 // Represents "Present Load" in Rot motors
#define ADDR_PRESENT_VELOCITY                  128
#define ADDR_PRESENT_POSITION                  132
#define VELOCITY_CONTROL                       1
#define DXL_LIMIT_MAX_VELOCITY                 330

// protocol version
#define PROTOCOL_VERSION                       2.0

// Motors ID
#define LEFT_DXL_ID                            0
#define RIGHT_DXL_ID                           1

/* TORQUE ENABLE/DISABLE */
#define TORQUE_ENABLE                   1	 // Value for enabling the torque
#define TORQUE_DISABLE                  0	 // Value for disabling the torque

// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyS5"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

/* Macro for Control Table Value */
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

// robot description
#define WHEEL_SEPARATION                0.196 //0.17153
#define WHEEL_RADIUS                    0.033
#define VELOCITY_CONSTANT_VAULE         1263.632956882

// math
#define PI                              3.14159265359

// motor pos
#define MOTOR_POS                       4096

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
dynamixel::GroupSyncRead *groupSyncReadEncoder_;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

class VelocityNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;
  using GetTwist = geometry_msgs::msg::Twist;
  using PubOdometry = nav_msgs::msg::Odometry;


  VelocityNode();
  virtual ~VelocityNode();

  void velocityCallBack(const std::shared_ptr<GetTwist> msg);

  bool writeVelocity(int64_t left_value, int64_t right_value);

private:
  rclcpp::Subscription<GetTwist>::SharedPtr velocity_subscriber_;
  rclcpp::Publisher<PubOdometry>::SharedPtr odom_publisher_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Time now_;
  rclcpp::Time base_time_;
  int32_t left_pos_, right_pos_, now_left_pos_, now_right_pos_;
  float theta_, x_, y_;
};

#endif
