#include "../include/teleoperation_dual_control/teleoperation_dual_control.hpp"


TeleoperationDualControl::TeleoperationDualControl() : Node("teleoperation_dual_control")
{
  lx = 0.0;
  ly = 0.0;
  rx = 0.0;
  ry = 0.0;

  running.store(true);



  publisher_teleoperation = this->create_publisher<std_msgs::msg::Float64MultiArray>("/teleoperation_dual_control/commands", 10);
}

void TeleoperationDualControl::publishPosition()
{
  auto msg_camera_position = std_msgs::msg::Float64MultiArray();
  msg_camera_position.data = {lx, ly, rx, ry};

  publisher_teleoperation->publish(msg_camera_position);
}
