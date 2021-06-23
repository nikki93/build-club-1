#pragma once
#include "core.hh"


//
// State
//

inline struct EditState {
  Prop(bool, enabled) { true };
  Prop(char[16], mode) { "select" };

  Prop(rl::Camera2D, camera) { {
      .target = { 0.5 * 800, 0.5 * 450 },
      .zoom = 1,
  } };
  Prop(float, zoomLevel) { 0 };
  float lineThickness = 1;

  char notification[96] = "";
  double lastNotificationTime = 0;

  Prop(char[64], inspectedComponentTitle) { "" };
} edit;


//
// Components
//

struct EditSelect {};

struct EditDelete {};

struct EditBox {
  rl::Rectangle rect;
};

struct EditMove {
  Vec2 delta { 0, 0 };
};


//
// Interface
//

// Mode

inline bool isEditMode(const char *mode) {
  return !std::strcmp(edit.mode(), mode);
}

inline void setEditMode(const char *newMode) {
  copy(edit.mode(), newMode);
}


// Boxes

inline void mergeEditBox(Entity ent, rl::Rectangle rect) {
  if (has<EditBox>(ent)) {
    auto &box = get<EditBox>(ent);
    Vec2 min {
      std::min(box.rect.x, rect.x),
      std::min(box.rect.y, rect.y),
    };
    Vec2 max {
      std::max(box.rect.x + box.rect.width, rect.x + rect.width),
      std::max(box.rect.y + box.rect.height, rect.y + rect.height),
    };
    box.rect.x = min.x;
    box.rect.y = min.y;
    box.rect.width = max.x - box.rect.x;
    box.rect.height = max.y - box.rect.y;
  } else {
    add<EditBox>(ent, { rect });
  }
}


// Camera

void setEditZoomLevel(float newZoomLevel);


// Session

void saveEditSnapshot(const char *desc, bool saveInspectedComponentTitle = false);
bool canUndoEdit();
void undoEdit();
bool canRedoEdit();
void redoEdit();

bool loadEditSession();

void playEdit();
void stopEdit();


// UI

void notifyEdit(const char *format, auto &&...args) {
  bprint(edit.notification, format, std::forward<decltype(args)>(args)...);
  edit.lastNotificationTime = rl::GetTime();
}

struct EditInspectContext {
  Entity ent = nullEntity;
  const char *componentTitle = "";
  bool changed = false;
  char changeDescription[96] = "";
  EditInspectAttribs attribs;
};


//
// Top-level
//

void updateEdit();
void drawEdit();
void uiEdit();
