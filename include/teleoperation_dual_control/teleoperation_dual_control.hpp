#ifndef BUILD_TELEOPERATION_DUAL_CONTROL_HPP
#define BUILD_TELEOPERATION_DUAL_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <thread>

#include <atomic>

#include "../../src/SDLWindow.cpp"


class TeleoperationDualControl : public rclcpp::Node
{
public:
  TeleoperationDualControl();
  ~TeleoperationDualControl() override;
  virtual void checkButton() = 0;
  void publishPosition();

 protected:
  double lx;
  double ly;
  double rx;
  double ry;

  std::thread keyboard_thread;

  SDLWindow* window;

 private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_teleoperation;
  rclcpp::TimerBase::SharedPtr check_buttons;
};


#endif //BUILD_TELEOPERATION_DUAL_CONTROL_HPP
