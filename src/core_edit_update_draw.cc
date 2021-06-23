#include "core.hh"

#include "game.hh"


//
// Update
//

void setEditZoomLevel(float newZoomLevel) {
  edit.zoomLevel() = newZoomLevel;
  if (edit.zoomLevel() == 0) {
    notifyEdit("zoomed 1x");
  } else if (edit.zoomLevel() > 0) {
    notifyEdit("zoomed in %.3gx", std::pow(1.5, edit.zoomLevel()));
  } else {
    notifyEdit("zoomed out %.3gx", 1 / std::pow(1.5, edit.zoomLevel()));
  }
}

void updateEdit() {
  // Mouse coordinates
  static Vec2 mouseScreenPos;
  auto prevMouseScreenPos = mouseScreenPos;
  mouseScreenPos = rl::GetMousePosition();
  static Vec2 mouseWorldPos;
  auto prevMouseWorldPos = mouseWorldPos;
  mouseWorldPos = rl::GetScreenToWorld2D(mouseScreenPos, edit.camera());
  auto mouseWorldDelta = mouseWorldPos - prevMouseWorldPos;

  // Select
  if (isEditMode("select")) {
    if (rl::IsMouseButtonReleased(0)) {
      // Collect hits in ascending area order
      constexpr auto maxNumHits = 32;
      struct Hit {
        Entity ent;
        float area;
      } hits[maxNumHits];
      auto numHits = 0;
      each([&](Entity ent, EditBox &box) {
        if (numHits < maxNumHits && rl::CheckCollisionPointRec(mouseWorldPos, box.rect)) {
          auto &hit = hits[numHits++];
          hit.ent = ent;
          hit.area = box.rect.width * box.rect.height;
        }
      });
      std::sort(hits, hits + numHits, [](const Hit &a, const Hit &b) {
        return a.area < b.area;
      });

      // Pick after current selection, or first if none
      Entity pick = nullEntity;
      auto pickNext = true;
      for (auto i = 0; i < numHits; ++i) {
        auto ent = hits[i].ent;
        if (has<EditSelect>(ent)) {
          pickNext = true;
        } else if (pickNext) {
          pick = ent;
          pickNext = false;
        }
      }
      each(remove<EditSelect>);
      if (pick != nullEntity) {
        add<EditSelect>(pick);
      }
    }
  }

  // Camera pan
  {
    static char prevMode[sizeof(edit.mode())] = "select";
    if (rl::IsMouseButtonPressed(2)) {
      std::memcpy(prevMode, edit.mode(), sizeof(prevMode));
      setEditMode("camera pan");
    }
    if (rl::IsMouseButtonReleased(2)) {
      std::memcpy(edit.mode(), prevMode, sizeof(prevMode));
    }
    if (isEditMode("camera pan")) {
      if ((!rl::IsMouseButtonPressed(0) && rl::IsMouseButtonDown(0)) || rl::IsMouseButtonDown(2)) {
        auto delta = mouseWorldPos - rl::GetScreenToWorld2D(prevMouseScreenPos, edit.camera());
        edit.camera().target -= delta;
      }
    }
  }

  // Move
  each(remove<EditMove>);
  if (isEditMode("move")) {
    if (!rl::IsMouseButtonPressed(0) && rl::IsMouseButtonDown(0)) {
      each([&](Entity ent, EditSelect &sel, EditBox &box) {
        auto &move = add<EditMove>(ent);
        move.delta += mouseWorldDelta;
      });
    }
    if (rl::IsMouseButtonReleased(0)) {
      saveEditSnapshot("move");
    }
  }

  // Clear boxes (game will add them every frame)
  each(remove<EditBox>);

  // Camera zoom and offset
  {
    static float wheelMove = 0;
    wheelMove += rl::GetMouseWheelMove();
    if (std::abs(wheelMove) > 0.1) {
      if (wheelMove < 0) {
        setEditZoomLevel(edit.zoomLevel() + 1);
      } else {
        setEditZoomLevel(edit.zoomLevel() - 1);
      }
      wheelMove = 0;
    }
    auto baseZoom = float(rl::GetScreenWidth()) / 800;
    edit.camera().zoom = float(baseZoom * std::pow(1.5, edit.zoomLevel()));
  }
  edit.camera().offset = 0.5 * Vec2 { float(rl::GetScreenWidth()), float(rl::GetScreenHeight()) };

  // Line thickness
  {
#ifdef __EMSCRIPTEN__
    static auto devicePixelRatio = []() {
      return float(EM_ASM_DOUBLE({ return window.devicePixelRatio; }));
    }();
#else
    static auto devicePixelRatio = rl::GetWindowScaleDPI().x;
#endif
    edit.lineThickness = devicePixelRatio / edit.camera().zoom;
  }

  // Game hook
  updateGameEdit();
}


//
// Draw
//

void drawEdit() {
  const auto drawRect = [&](rl::Rectangle rect, rl::Color color) {
    Vec2 v0 { rect.x, rect.y };
    Vec2 v1 { v0.x + rect.width, v0.y };
    Vec2 v2 { rect.x + rect.width, rect.y + rect.height };
    Vec2 v3 { v0.x, v0.y + rect.height };
    rl::DrawLineEx(v0, v1, edit.lineThickness, color);
    rl::DrawLineEx(v1, v2, edit.lineThickness, color);
    rl::DrawLineEx(v2, v3, edit.lineThickness, color);
    rl::DrawLineEx(v3, v0, edit.lineThickness, color);
  };

  // Red boxes for unselected entities
  if (isEditMode("select")) {
    each([&](Entity ent, EditBox &box) {
      if (!has<EditSelect>(ent)) {
        drawRect(box.rect, rl::RED);
      }
    });
  }

  // Doubled green boxes for selected entities
  if (isEditMode("select") || isEditMode("move")) {
    auto camTopLeft = rl::GetScreenToWorld2D({ 0, 0 }, edit.camera());
    Vec2 screenSize { float(rl::GetScreenWidth()), float(rl::GetScreenHeight()) };
    auto camBottomRight = rl::GetScreenToWorld2D(screenSize, edit.camera());
    auto border = 2 * edit.lineThickness;
    rl::Rectangle camRect {
      camTopLeft.x + border,
      camTopLeft.y + border,
      camBottomRight.x - camTopLeft.x - 2 * border,
      camBottomRight.y - camTopLeft.y - 2 * border,
    };
    each([&](Entity ent, EditSelect &sel, EditBox &box) {
      auto rect = rl::GetCollisionRec(camRect, box.rect);
      drawRect(rect, { 0, 0x80, 0x40, 0xff });
      rl::Rectangle biggerRect {
        rect.x - border,
        rect.y - border,
        rect.width + 2 * border,
        rect.height + 2 * border,
      };
      drawRect(biggerRect, { 0, 0x80, 0x40, 0xff });
    });
  }

  // Game hook
  drawGameEdit();
}
