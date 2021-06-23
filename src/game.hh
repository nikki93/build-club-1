#pragma once
#include "core.hh"


//
// Constants
//



//
// State
//

inline struct GameState {
  Prop(double, time) { 0 };

  Prop(Vec2, viewSize) { 800, 450 };
  Prop(rl::Camera2D, camera) { {
      .target = { 0.5f * viewSize().x, 0.5f * viewSize().y },
      .zoom = 1,
  } };
} game;



//
// Components
//



//
// Top-level
//

void initGame();
void updateGame(float dt);
void drawGame();

void updateGameEdit();
void drawGameEdit();
