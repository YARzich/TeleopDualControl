#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>
#include <iostream>

class SDLWindow {
 public:
  SDLWindow(const std::string& windowTitle, int width, int height)
      : window(nullptr), renderer(nullptr), font(nullptr)
      {
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
      std::cerr << "Initialization error SDL: " << SDL_GetError() << std::endl;
      return;
    }

    if (TTF_Init() == -1)
    {
      std::cerr << "Initialization error SDL_ttf: " << TTF_GetError() << std::endl;
      SDL_Quit();
      return;
    }

    window = SDL_CreateWindow(windowTitle.c_str(),
                              SDL_WINDOWPOS_CENTERED,
                              SDL_WINDOWPOS_CENTERED,
                              width, height, SDL_WINDOW_SHOWN);
    if (!window)
    {
      std::cerr << "Window creation error: " << SDL_GetError() << std::endl;
      TTF_Quit();
      SDL_Quit();
      return;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer)
    {
      std::cerr << "Rendering creation error: " << SDL_GetError() << std::endl;
      SDL_DestroyWindow(window);
      TTF_Quit();
      SDL_Quit();
      return;
    }

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);
  }

  ~SDLWindow()
  {
    if (font)
    {
      TTF_CloseFont(font);
    }
    if (renderer)
    {
      SDL_DestroyRenderer(renderer);
    }
    if (window)
    {
      SDL_DestroyWindow(window);
    }
    TTF_Quit();
    SDL_Quit();
  }

  void loadFont(const std::string& fontPath, int fontSize)
  {
    font = TTF_OpenFont(fontPath.c_str(), fontSize);
    if (!font)
    {
      throw std::runtime_error("The font could not be loaded" + std::string(TTF_GetError()));
    }
  }

  void renderText(const std::string& text)
  {
    SDL_Color color = { 255, 255, 255, 255 };

    if (!font)
    {
      throw std::runtime_error("The font is not loaded");
    }

    SDL_Surface* textSurface = TTF_RenderText_Solid(font, text.c_str(), color);
    if (!textSurface)
    {
      throw std::runtime_error("Failed to create a text surface: " + std::string(TTF_GetError()));
    }

    SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);
    if (!textTexture)
    {
      SDL_FreeSurface(textSurface);
      throw std::runtime_error("Failed to create texture: " + std::string(SDL_GetError()));
    }

    int textWidth = textSurface->w;
    int textHeight = textSurface->h;
    SDL_FreeSurface(textSurface);

    int windowWidth, windowHeight;
    SDL_GetWindowSize(window, &windowWidth, &windowHeight);
    SDL_Rect textRect = { (windowWidth - textWidth) / 2,
                          (windowHeight - textHeight) / 2,
                          textWidth,
                          textHeight };

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    SDL_RenderCopy(renderer, textTexture, nullptr, &textRect);

    SDL_RenderPresent(renderer);

    SDL_DestroyTexture(textTexture);
  }

 private:
  SDL_Window* window;
  SDL_Renderer* renderer;
  TTF_Font* font;
};
