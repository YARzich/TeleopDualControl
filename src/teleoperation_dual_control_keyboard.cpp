#include "teleoperation_dual_control.cpp"

#include <SDL2/SDL.h>


class TeleoperationDualControlKeyboard : public TeleoperationDualControl
{
public:
  SDL_Window* window;
  TeleoperationDualControlKeyboard() : TeleoperationDualControl()
  {
    if (SDL_Init(SDL_INIT_EVENTS | SDL_INIT_VIDEO) != 0)
    {
      throw std::runtime_error("Failed to initialize SDL: " + std::string(SDL_GetError()));
    }

    window = SDL_CreateWindow("Hidden Window",
                                          SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                          640, 480, SDL_WINDOW_SHOWN);
    if (!window) {
      std::cerr << "Не удалось создать скрытое окно: " << SDL_GetError() << std::endl;
    }

    keyboard_thread = std::thread(&TeleoperationDualControlKeyboard::getPosition, this);

    RCLCPP_INFO(this->get_logger(), "Teleoperation Dual Control started");
  }

  ~TeleoperationDualControlKeyboard() override
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down");
    running.store(false);
    SDL_DestroyWindow(window);
    SDL_Quit();
  }

  void getPosition() override
  {
    const Uint8* key_state = SDL_GetKeyboardState(nullptr);

    while (running.load())
    {
      SDL_PumpEvents();
      if (key_state[SDL_SCANCODE_ESCAPE]) {
        running.store(false);
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
      SDL_Delay(100);
      lx = 0.0;
      ly = 0.0;
      rx = 0.0;
      ry = 0.0;
    }
    delete this;
  }
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "Teleoperation Dual Control" << std::endl;
  rclcpp::spin(std::make_shared<TeleoperationDualControlKeyboard>());
  rclcpp::shutdown();
  return 0;
}