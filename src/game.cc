#include "game.hh"


constexpr float ballRadius = 10;

constexpr float paddleY = 0.9 * tableSize.y;
constexpr Vec2 paddleSize { 60, 12 };

constexpr Vec2 brickSize { 50, 20 };
constexpr int numBricksX = 10;
constexpr int numBricksY = 5;

void initGame() {
  if (!loadEditSession()) {
    auto xIncrement = tableSize.x / (numBricksX + 1);
    for (auto i = 0; i < numBricksX; ++i) {
      auto x = xIncrement * (i + 1);
      game.bricks().push_back({
          .pos { x, 60 },
      });
    }
  }
}

float screenScale = 1;

void updateGame(float dt) {
  dprint("fps: %d", rl::GetFPS());

  // Update screen scale
  screenScale = rl::GetScreenWidth() / tableSize.x;

  // Move paddle with mouse
  game.paddleX() = rl::GetMousePosition().x / screenScale;

  // Apply velocity to ball
  game.ballPos() += game.ballVel() * dt;

  // Bounce ball against walls
  if (game.ballPos().y < ballRadius) {
    game.ballPos().y = ballRadius;
    game.ballVel().y *= -1;
  }
  if (game.ballPos().x < ballRadius) {
    game.ballPos().x = ballRadius;
    game.ballVel().x *= -1;
  }
  if (game.ballPos().y > tableSize.y - ballRadius) {
    game.ballPos().y = tableSize.y - ballRadius;
    game.ballVel().y *= -1;
  }
  if (game.ballPos().x > tableSize.x - ballRadius) {
    game.ballPos().x = tableSize.x - ballRadius;
    game.ballVel().x *= -1;
  }

  // Bounce ball against paddle
  {
    Vec2 paddlePos { game.paddleX(), paddleY };
    auto paddleTopLeft = paddlePos - 0.5 * paddleSize;
    rl::Rectangle paddleRect { paddleTopLeft.x, paddleTopLeft.y, paddleSize.x, paddleSize.y };
    if (rl::CheckCollisionCircleRec(game.ballPos(), ballRadius, paddleRect)) {
      auto delta = game.ballPos() - paddlePos;
      delta /= paddleSize;
      game.ballPos() -= game.ballVel() * dt;
      if (std::abs(delta.x) > std::abs(delta.y)) {
        game.ballVel().x *= -1;
      } else {
        game.ballVel().y *= -1;
      }
    }
  }

  // Bounce ball against bricks + destroy bricks
  auto iter
      = std::remove_if(game.bricks().begin(), game.bricks().end(), [&](GameState::Brick &brick) {
          auto topLeft = brick.pos() - 0.5 * brickSize;
          rl::Rectangle rect { topLeft.x, topLeft.y, brickSize.x, brickSize.y };
          if (rl::CheckCollisionCircleRec(game.ballPos(), ballRadius, rect)) {
            auto delta = game.ballPos() - topLeft;
            delta /= brickSize;
            game.ballPos() -= game.ballVel() * dt;
            if (std::abs(delta.x) > std::abs(delta.y)) {
              game.ballVel().x *= -1;
            } else {
              game.ballVel().y *= -1;
            }
            return true;
          }
          return false;
        });
  game.bricks().erase(iter, game.bricks().end());
}

void drawGame() {
  rl::PushMatrix();

  // Apply screen scale
  rl::Scalef(screenScale, screenScale, 1);

  // Draw ball
  rl::DrawCircle(game.ballPos().x, game.ballPos().y, ballRadius, rl::RED);

  // Draw paddle
  Vec2 paddlePos { game.paddleX(), paddleY };
  auto paddleTopLeft = paddlePos - 0.5 * paddleSize;
  rl::DrawRectangle(paddleTopLeft.x, paddleTopLeft.y, paddleSize.x, paddleSize.y, rl::BLUE);

  // Draw bricks
  for (auto &brick : game.bricks()) {
    auto topLeft = brick.pos() - 0.5 * brickSize;
    rl::DrawRectangle(topLeft.x, topLeft.y, brickSize.x, brickSize.y, rl::YELLOW);
  }

  rl::PopMatrix();
}
