#include <string>

#include "../include/teleoperation_dual_control/teleoperation_dual_control.hpp"


TeleoperationDualControl::TeleoperationDualControl() : Node("teleoperation_dual_control"),
lx(0.0), ly(0.0), rx(0.0), ry(0.0)
{
  publisher_teleoperation = this->create_publisher<std_msgs::msg::Float64MultiArray>("/teleoperation_dual_control/commands", 10);
  check_buttons = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this] { checkButton(); }
  );

  std::string name_window =
      get_parameter_or("name_window", rclcpp::Parameter("name_window", "Robot teleoperation"))
      .get_value<std::string>();

  int width = get_parameter_or("width", rclcpp::Parameter("width", 800)).get_value<int>();
  int height = get_parameter_or("height", rclcpp::Parameter("height", 600)).get_value<int>();

  window = new SDLWindow(name_window, width, height);
  window->loadFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24);
}

TeleoperationDualControl::~TeleoperationDualControl()
{
  delete window;
  RCLCPP_INFO(this->get_logger(), "Shutting down");
}

void TeleoperationDualControl::publishPosition()
{
  auto msg_camera_position = std_msgs::msg::Float64MultiArray();
  msg_camera_position.data = {lx, ly, rx, ry};

  publisher_teleoperation->publish(msg_camera_position);

  window->renderText(std::to_string(lx) + " " + std::to_string(ly) + " "
  + std::to_string(rx) + " " + std::to_string(ry));
}
