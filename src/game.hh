#pragma once
#include "core.hh"


//
// Constants
//


//
// State
//

inline struct GameState {
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
void uiGame();

void updateGameEdit();
void drawGameEdit();
