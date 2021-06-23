#include "core.hh"

#include "game.hh"
UseComponentTypes();


//
// Toolbar
//

static void uiEditToolbar() {
  // Left
  {
    // Play / stop
    ui("button")(edit.enabled() ? "play" : "stop")(
        "title", edit.enabled() ? "play game (spacebar)" : "stop game (spacebar)")("click",
        rl::KEY_SPACE, [&]() {
          if (edit.enabled()) {
            playEdit();
          } else {
            stopEdit();
          }
        });
    if (!edit.enabled()) {
      return;
    }
  }

  auto hasSelection = false;
  each([&](Entity ent, EditSelect &sel) {
    hasSelection = true;
  });
  if (!hasSelection && !(isEditMode("select") || isEditMode("camera pan"))) {
    setEditMode("select");
  }

  // Center
  ui("div")("flex-gap");
  {
    ui("div")([&]() {
      if (hasSelection) {
        // Move
        ui("button")("move")(
            "selected", isEditMode("move"))("title", "move (m)")("click", rl::KEY_M, [&]() {
          setEditMode(isEditMode("move") ? "select" : "move");
        });
      }
    });
  }
  ui("div")("flex-gap");

  // Right
  {
    ui("div")([&]() {
      if (hasSelection) {
        // Delete
        ui("button")("delete")("title", "delete (backspace)")("click", rl::KEY_BACKSPACE, [&]() {
          each([&](Entity ent, EditSelect &sel) {
            add<EditDelete>(ent);
          });
          saveEditSnapshot("delete");
          each([&](Entity ent, EditDelete &del) {
            destroyEntity(ent);
          });
        });
      }
    });
    ui("div")("small-gap");

    // Undo / redo
    ui("button")("undo")("disabled", !canUndoEdit())("title", "undo (ctrl+z)")("click",
        rl::KEY_LEFT_CONTROL, rl::KEY_Z, [&]() {
          undoEdit();
        });
    ui("button")("redo")("disabled", !canRedoEdit())("title", "redo (ctrl+y)")("click",
        rl::KEY_LEFT_CONTROL, rl::KEY_Y, [&]() {
          redoEdit();
        });
    ui("div")("small-gap");

    // Pan
    ui("button")("pan")(
        "selected", isEditMode("camera pan"))("title", "pan view (p)")("click", rl::KEY_P, [&]() {
      setEditMode(isEditMode("camera pan") ? "select" : "camera pan");
    });

    // Zoom
    ui("button")("zoom-in")("title", "zoom in (= or scroll up)")("click", rl::KEY_EQUAL, [&]() {
      setEditZoomLevel(edit.zoomLevel() + 1);
    });
    ui("button")("zoom-out")("title", "zoom out (- or scroll down)")("click", rl::KEY_MINUS, [&]() {
      setEditZoomLevel(edit.zoomLevel() - 1);
    });
  }
}


//
// Inspect
//

// int
static void inspect(int &val, EditInspectContext &ctx) {
  static char str[64];
  bprint(str, "%d", val);
  ui("input")("type", "number")("step", "any")("value", str)("change", [&](const char *newStr) {
    auto oldVal = val;
    std::sscanf(newStr, "%d", &val);
    ctx.changed = val != oldVal;
    return bprint(str, "%d", val);
  });
}

// double
static void inspect(double &val, EditInspectContext &ctx) {
  static char str[64];
  const auto printToStr = [&]() {
    bprint(str, "%.4f", val);
    for (auto i = int(std::strlen(str)) - 1; i > 0; --i) {
      if (str[i] != '0') {
        if (str[i] == '.') {
          --i;
        }
        str[i + 1] = '\0';
        break;
      }
    }
  };
  printToStr();
  ui("input")("type", "number")("step", "any")("value", str)("change", [&](const char *newStr) {
    auto oldVal = val;
    std::sscanf(newStr, "%lf", &val);
    ctx.changed = val != oldVal;
    printToStr();
    return str;
  });
}

// float
static void inspect(float &val, EditInspectContext &ctx) {
  double d = val;
  inspect(d, ctx);
  val = float(d);
}

// bool
static void inspect(bool &val, EditInspectContext &ctx) {
  ui("button")(val ? "on" : "off")("click", [&]() {
    val = !val;
    ctx.changed = true;
  });
}

// char array
template<int N>
static void inspect(char (&val)[N], EditInspectContext &ctx) {
  ui("input")("value", val)("change", [&](const char *newVal) {
    if (std::strcmp(val, newVal) != 0) {
      copy(val, newVal);
      ctx.changed = true;
    }
    return val;
  });
}

// Vec2
static void inspect(Vec2 &val, EditInspectContext &ctx) {
  ui("div")("numbers-row")([&]() {
    inspect(val.x, ctx);
    inspect(val.y, ctx);
  });
}

// rl::Rectangle
static void inspect(rl::Rectangle &val, EditInspectContext &ctx) {
  ui("div")("numbers-row")([&]() {
    ui("div")("numbers-row")([&]() {
      inspect(val.x, ctx);
      inspect(val.y, ctx);
    });
    ui("div")("numbers-row")([&]() {
      inspect(val.width, ctx);
      inspect(val.height, ctx);
    });
  });
}

// Seq<T, N>
static int inspectActiveSeqMenuToken = -1;
template<typename T, unsigned N>
static void inspect(Seq<T, N> &val, EditInspectContext &ctx) {
  auto nElems = int(val.size());
  auto removeI = -1, swapNextI = -1, addBeforeI = -1;
  auto i = 0;
  for (auto &elem : val) {
    ui("div")("seq-elem-container")([&]() {
      auto token = JS_uiGetElemToken();
      ui("div")([&]() {
        ui("button")("show-seq-elem-menu")("click", [&]() {
          inspectActiveSeqMenuToken = token;
        });
      });
      ui("div")("seq-elem-menu-anchor")([&]() {
        inspect(elem, ctx);
        if (inspectActiveSeqMenuToken == token) {
          ui("div")("seq-elem-menu-background")("click", [&]() {
            inspectActiveSeqMenuToken = -1;
          });
          ui("div")("seq-elem-menu-container")([&]() {
            ui("button")("remove")("label", "remove item")("click", [&]() {
              removeI = i;
            });
            if (i > 0) {
              ui("button")("up")("label", "move up")("click", [&]() {
                swapNextI = i - 1;
              });
            }
            if (i < nElems - 1) {
              ui("button")("down")("label", "move down")("click", [&]() {
                swapNextI = i;
              });
            }
            ui("button")("add")("label", "add before")("click", [&]() {
              addBeforeI = i;
            });
          });
        }
      });
    });
    ++i;
  }
  if (removeI >= 0) {
    val.erase(val.begin() + removeI);
    inspectActiveSeqMenuToken = -1;
    ctx.changed = true;
  }
  if (swapNextI >= 0) {
    std::swap(val[swapNextI], val[swapNextI + 1]);
    inspectActiveSeqMenuToken = -1;
    ctx.changed = true;
  }
  if (addBeforeI >= 0) {
    val.insert(val.begin() + addBeforeI, T {});
    inspectActiveSeqMenuToken = -1;
    ctx.changed = true;
  }
  ui("div")("seq-elem-container")([&]() {
    ui("div")([&]() {
      ui("button")("show-seq-elem-menu")("disabled", true);
    });
    ui("button")("add-seq-elem")("click", [&]() {
      val.emplace_back();
      ctx.changed = true;
    });
  });
}

// Props
template<typename Value, typename Internal>
static void inspect(PropHolder<Value, Internal> &prop, EditInspectContext &ctx) {
  inspect(prop(), ctx);
}
static void inspect(auto &val, EditInspectContext &ctx) {
  ui("div")("props-container")([&]() {
    forEachProp(val, [&]<typename P>(P &prop) {
      if constexpr (P::attribs.inspect.breakBefore) {
        ui("div")("prop-break");
      }
      ui("div")("prop-container")([&]() {
        ui("div")("prop-name")([&]() {
          uiText(P::name.data());
        });
        ui("div")("prop-value-container")([&]() {
          auto prevChanged = ctx.changed;
          auto prevAttribs = ctx.attribs;
          ctx.attribs = P::attribs.inspect;
          inspect(prop, ctx);
          ctx.attribs = prevAttribs;
          if constexpr (requires { set(prop, ctx.ent); }) {
            if (ctx.changed && !prevChanged) {
              set(prop, ctx.ent);
            }
          }
          if (ctx.changed && isEmpty(ctx.changeDescription)) {
            bprint(ctx.changeDescription, "change %s %s", ctx.componentTitle, P::name.data());
          }
        });
      });
    });
  });
}

// Inspector
static void uiEditInspector() {
  static const auto titleify = [&](std::string_view title) {
    static char buf[64];
    auto i = 0;
    for (auto c : title) {
      if (i >= int(sizeof(buf) - 2)) {
        break;
      }
      if (std::isupper(c)) {
        if (i > 0) {
          buf[i++] = ' ';
        }
        buf[i++] = char(std::tolower(c));
      } else {
        buf[i++] = c;
      }
    }
    buf[i] = '\0';
    return buf;
  };

  each([&](Entity ent, EditSelect &sel) {
    char nextInspectedComponentTitle[sizeof(edit.inspectedComponentTitle())] = "";
    void (*remover)(Entity ent) = nullptr;

    // Section for each component entity has
    forEachComponentType([&]<typename T>(T *) {
      if (has<T>(ent)) {
        auto title = titleify(getTypeName<T>());
        auto open = !std::strcmp(edit.inspectedComponentTitle(), title);
        ui("details", title)(title)("open", open)([&]() {
          // Title
          ui("summary")("click", [&]() {
            if (open) {
              edit.inspectedComponentTitle()[0] = '\0';
            } else {
              copy(nextInspectedComponentTitle, title);
            }
          })([&]() {
            uiText(title);
            ui("button")("remove")("click", [&]() {
              remover = [](Entity ent) {
                remove<T>(ent);
                auto title = titleify(getTypeName<T>());
                copy(edit.inspectedComponentTitle(), title);
                saveEditSnapshot(bprint<64>("remove %s", title), true);
              };
            });
          });

          // Properties
          EditInspectContext ctx { .ent = ent, .componentTitle = title };
          auto &comp = get<T>(ent);
          ui("div")("component-container")([&]() {
            inspect(comp, ctx);
          });
          if (!isEmpty(ctx.changeDescription)) {
            saveEditSnapshot(ctx.changeDescription, true);
          } else if (ctx.changed) {
            saveEditSnapshot(bprint<96>("edit %s", title), true);
          }
        });
      }
    });

    // Buttons to add components entity doesn't have
    ui("div")("add-bar")([&]() {
      forEachComponentType([&]<typename T>(T *) {
        if (!has<T>(ent)) {
          auto title = titleify(getTypeName<T>());
          ui("button")("add")("label", title)("click", [&]() {
            add<T>(ent);
            copy(edit.inspectedComponentTitle(), title);
            saveEditSnapshot(bprint<64>("add %s", title), true);
          });
        }
      });
    });

    // Doing these after UI calls to prevent rendering insconsistent state
    if (!isEmpty(nextInspectedComponentTitle)) {
      copy(edit.inspectedComponentTitle(), nextInspectedComponentTitle);
    }
    if (remover) {
      remover(ent);
    }
  });
}


//
// Status
//

static void uiEditStatus() {
  // FPS
  ui("div")([&]() {
    uiText("fps: %d", rl::GetFPS());
  });

  if (edit.enabled()) {
    ui("div")("flex-gap");

    // Notification
    if (!isEmpty(edit.notification)) {
      if (rl::GetTime() - edit.lastNotificationTime < 3.5) {
        ui("div")([&]() {
          uiText(edit.notification);
        });
        ui("div")("small-gap");
      } else {
        edit.notification[0] = '\0';
      }
    }

    // Mode
    ui("div")([&]() {
      uiText(edit.mode());
    });
  }
}


//
// Top-level
//

void uiEdit() {
  uiPatch("top", [&]() {
    ui("div")("toolbar")([&]() {
      uiEditToolbar();
    });
  });
  uiPatch("side", [&]() {
    ui("div")("inspector")([&]() {
      uiEditInspector();
    });
  });
  uiPatch("bottom", [&]() {
    ui("div")("status")([&]() {
      uiEditStatus();
    });
  });
}
