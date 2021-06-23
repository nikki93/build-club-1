#include "core.hh"
#include "game.hh"


// Run a single frame of the main loop
void frame() {
  // Window size
  Vec2 windowSize { float(rl::GetScreenWidth()), float(rl::GetScreenHeight()) };
  {
#ifdef __EMSCRIPTEN__
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

  // UI
  uiEdit();

  // Update
  if (edit.enabled()) {
    updateEdit();
  } else {
    updateGame(std::min(rl::GetFrameTime(), 0.06f));
  }

  // Draw
  rl::BeginDrawing();
  {
    rl::ClearBackground({ 0xcc, 0xe4, 0xf5, 0xff });

    game.camera().offset = 0.5 * windowSize;
    game.camera().zoom = float(windowSize.x) / 800;
    rl::BeginMode2D(edit.enabled() ? edit.camera() : game.camera());
    drawGame();
    if (edit.enabled()) {
      drawEdit();
    }
    rl::EndMode2D();

    rl::DrawText(debugDisplayBuffer, 10, 10, 24, rl::BLACK);
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
