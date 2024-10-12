#include "teleoperation_dual_control.cpp"


class TeleoperationDualControlKeyboard : public TeleoperationDualControl
{
public:
  TeleoperationDualControlKeyboard() : TeleoperationDualControl()
  {
    key_state = SDL_GetKeyboardState(nullptr);

    RCLCPP_INFO(this->get_logger(), "Teleoperation Dual Control Keyboard started");
  }

  void checkButton() override
  {
    SDL_PumpEvents();
    if (key_state[SDL_SCANCODE_ESCAPE]) {
      rclcpp::shutdown();
    }

    if (key_state[SDL_SCANCODE_W]) {
      ly += 1.0;
    }

    if (key_state[SDL_SCANCODE_S]) {
      ly += -1.0;
    }

    if (key_state[SDL_SCANCODE_A]) {
      lx += 1.0;
    }

    if (key_state[SDL_SCANCODE_D]) {
      lx += -1.0;
    }

    if (key_state[SDL_SCANCODE_UP]) {
      ry += 1.0;
    }

    if (key_state[SDL_SCANCODE_DOWN]) {
      ry += -1.0;
    }

    if (key_state[SDL_SCANCODE_LEFT]) {
      rx += 1.0;
    }

    if (key_state[SDL_SCANCODE_RIGHT]) {
      rx += -1.0;
    }

    publishPosition();
    lx = 0.0;
    ly = 0.0;
    rx = 0.0;
    ry = 0.0;
  }

 private:
  const Uint8* key_state;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleoperationDualControlKeyboard>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}