#include "motor_driver/motor_control.hpp"

VelocityNode::VelocityNode()
: Node("velocity_node"), base_time_(rclcpp::Time(0, 0, this->get_clock()->get_clock_type())), x_(0.0), y_(0.0), theta_(0.0), now_left_pos_(0.0), now_right_pos_(0.0)
{
  RCLCPP_INFO(this->get_logger(), "Run velocity node");
  // qos setting
  this->declare_parameter("qos_depth", 100);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);
  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
  
  
  // open config file
  // this->declare_parameter("config_path", "");
  // std::string config_path;
  // this->get_parameter("config_path", config_path);
  // YAML::Node node = YAML::LoadFile(config_path);
  // YAML::Node dym_config = node["dynamixel_config"];
  
  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  // subscriber
  velocity_subscriber_ =
    this->create_subscription<GetTwist>(
      "cmd_vel",
      QOS_RKL10V,
      std::bind(&VelocityNode::velocityCallBack,
      this,
      std::placeholders::_1));
  
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

VelocityNode::~VelocityNode()
{
}


void  VelocityNode::velocityCallBack(const std::shared_ptr<GetTwist> msg)
{
  float  right_velocity, left_velocity;
  left_velocity =  ((msg->linear.x - (msg->angular.z * WHEEL_SEPARATION / 2)) * 41.69988758) / WHEEL_RADIUS;
  right_velocity =  ((-1 * (msg->linear.x - (-1 * msg->angular.z * WHEEL_SEPARATION / 2))) * 41.69988758 / WHEEL_RADIUS);
  RCLCPP_INFO(get_logger(), "left_velocity: %f, right_velocity: %f", left_velocity, right_velocity);

  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, LEFT_DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&now_left_pos_, &dxl_error);
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, RIGHT_DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&now_right_pos_, &dxl_error);
  
  // left_data = (left_velocity * 41.69988758) / WHEEL_RADIUS;
  // right_data = (right_velocity * 41.69988758) / WHEEL_RADIUS;
  /*velocity_constant_value =  + 41.69988758; // V = r * w
                                         // w = ((0.229 * Goal_Velocity) * 3.14159265359 )/ 30
                                         // Goal_Velocity = (41.69988758 * V )/ r 
  */

  // TODO
  now_ = this->get_clock()->now();
  rclcpp::Duration duration_ = now_ - base_time_;
  float duration_time_ = duration_.seconds();
  //   // angular velocity
  
  //   float duration_time_ = duration_.seconds();
  //   RCLCPP_INFO(this->get_logger(), "Duration: %f seconds", duration_time_);
  //   float velocity_duration_;
  //   if (left_velocity > right_velocity) {
  //     velocity_duration_ = left_velocity - right_velocity;
  //   }
  //   else if (right_velocity > left_velocity) {
  //     velocity_duration_ = right_velocity - left_velocity;
  //   }
  //   else {
  //     velocity_duration_ = 0
  //   }
  //   float angular_velocity_ = velocity_duration_ / (2 * WHEEL_SEPARATION);
  //   theta_ = angular_velocity_ * duration_time_;}
  x_ += msg->linear.x * duration_time_ * cos(theta_);
  y_ += msg->linear.x * duration_time_ * sin(theta_);
  theta_ += msg->angular.z * duration_time_;

  // odom
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = now_;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  //tf
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->get_clock()->now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = x_;
  transform.transform.translation.y = y_;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();	

  // pub
  odom_publisher_->publish(odom_msg);
  tf_broadcaster_->sendTransform(transform);
  
  base_time_ = now_;
  
  dxl_comm_result = writeVelocity((int64_t)left_velocity, (int64_t)right_velocity);
}

bool VelocityNode::writeVelocity(int64_t left_value, int64_t right_value)
{
  bool dxl_addparam_result;

  uint8_t left_data_byte[4] = {0, };
  uint8_t right_data_byte[4] = {0, };


  left_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(left_value));
  left_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(left_value));
  left_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(left_value));
  left_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(left_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(LEFT_DXL_ID, (uint8_t*)&left_data_byte);
  if (dxl_addparam_result != true)
    return false;

  right_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(right_value));
  right_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(right_value));
  right_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(right_value));
  right_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(right_value));

  dxl_addparam_result = groupSyncWriteVelocity_->addParam(RIGHT_DXL_ID, (uint8_t*)&right_data_byte);
  if (dxl_addparam_result != true)
    return false;

  dxl_comm_result = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("velocity_node"), "Failed to sent data.");
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}


void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    VELOCITY_CONTROL,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    TORQUE_ENABLE,  /* Torque ON */
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<VelocityNode>();
  rclcpp::spin(readwritenode);

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    TORQUE_DISABLE,
    &dxl_error
  );
  portHandler->closePort();
	
	rclcpp::shutdown();

  return 0;
}
