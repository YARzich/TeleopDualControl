#ifndef BUILD_TELEOPERATION_DUAL_CONTROL_HPP
#define BUILD_TELEOPERATION_DUAL_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <thread>

#include <atomic>


class TeleoperationDualControl : public rclcpp::Node
{
public:
  TeleoperationDualControl();
  virtual void getPosition() = 0;
  void publishPosition();

 protected:
  double lx;
  double ly;
  double rx;
  double ry;

  std::atomic<bool> running;

  std::thread keyboard_thread;
 private:

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_teleoperation;
};


#endif //BUILD_TELEOPERATION_DUAL_CONTROL_HPP
