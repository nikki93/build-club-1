#include "core.hh"
#include "game.hh"


// Run a single frame of the main loop
void frame() {
  // Window size
  {
#ifdef __EMSCRIPTEN__
    Vec2 windowSize { float(rl::GetScreenWidth()), float(rl::GetScreenHeight()) };
    Vec2 canvasSize {
      float(int(EM_ASM_DOUBLE({
        return window.devicePixelRatio
            * document.querySelector("#canvas").getBoundingClientRect().width;
      }))),
      float(int(EM_ASM_DOUBLE({
        return window.devicePixelRatio
            * document.querySelector("#canvas").getBoundingClientRect().height;
      }))),
    };
    if (windowSize != canvasSize) {
      rl::SetWindowSize(int(canvasSize.x), int(canvasSize.y));
    }
#endif
  }

  // Pause on window unfocus on web
  {
#ifdef __EMSCRIPTEN__
    static auto firstFrame = true;
    if (!firstFrame) {
      static auto prevFocused = true;
      bool focused = EM_ASM_INT({ return document.hasFocus() ? 1 : 0; });
      if (focused != prevFocused) {
        prevFocused = focused;
        if (focused) {
          emscripten_set_main_loop_timing(EM_TIMING_RAF, 0);
        } else {
          emscripten_set_main_loop_timing(EM_TIMING_SETTIMEOUT, 100);
        }
      }
      if (!focused) {
        return;
      }
    }
    firstFrame = false;
#endif
  }

  // Debug display
  clearDebugDisplay();

  // Update
  updateGame(std::min(rl::GetFrameTime(), 0.06f));

  // Draw
  rl::BeginDrawing();
  {
    rl::ClearBackground({ 0x30, 0x30, 0x30, 0xff });

    drawGame();

    rl::DrawText(debugDisplayBuffer, 10, 10, 48, rl::WHITE);
  }
  rl::EndDrawing();
}


// Program entrypoint
int main() {
  initGame();

#ifdef __EMSCRIPTEN__
  emscripten_set_main_loop(frame, 0, true);
#else
  while (!rl::WindowShouldClose()) {
    frame();
  }
#endif

  return 0;
}
