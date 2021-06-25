#pragma once
#include "core.hh"


//
// Constants
//


//
// State
//

constexpr Vec2 tableSize { 800, 450 };

inline struct GameState {
  Prop(Vec2, ballPos) { 0.5f * tableSize.x, 0.8f * tableSize.y };
  Prop(Vec2, ballVel) { -160, -160 };

  Prop(float, paddleX) { 0.5 * tableSize.x };

  struct Brick {
    Prop(Vec2, pos) { 0, 0 };
  };
  Prop(Seq<Brick>, bricks);
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
