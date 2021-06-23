#include "core.hh"

#include "game.hh"
UseComponentTypes();


//
// History
//

struct EditComponentSnapshot {
  void *data;
  void (*free)(void *data);
  void (*add)(void *data, Entity ent);
  cJSON *(*write)(void *data);
};
struct EditEntitySnapshot {
  Prop((Seq<EditComponentSnapshot, 8>), componentSnapshots);
  Prop(Entity, ent);
};
struct EditSceneSnapshot {
  Prop(bool, valid) { false };
  Prop(double, time) { 0 };
  Prop(char[64], description);
  Prop(char[64], inspectedComponentTitle) { "" };
  Prop(Seq<EditEntitySnapshot>, entitySnapshots);
  Prop(Seq<Entity>, selection);
};
static constexpr auto editMaxSnapshots = 50;
static struct EditHistory {
  Prop(EditSceneSnapshot[editMaxSnapshots], snapshots);
  Prop(int, lastIndex) { 0 };
  Prop(Seq<Entity>, stopSelection);
} editHistory;


// Component snapshot

template<typename T>
void init(EditComponentSnapshot &componentSnapshot) {
  componentSnapshot.data = new T {};
  componentSnapshot.free = [](void *data) {
    delete ((T *)data);
  };
  componentSnapshot.add = [](void *data, Entity ent) {
    auto copy = *((T *)data);
    add<T>(ent, std::move(copy));
  };
  componentSnapshot.write = [](void *data) {
    auto &comp = *((T *)data);
    auto jsn = write(comp);
    cJSON_AddItemToObjectCS(jsn, "_type", cJSON_CreateString(nullTerminate<64>(getTypeName<T>())));
    return jsn;
  };
}

void read(EditComponentSnapshot &componentSnapshot, const cJSON *jsn) {
  const auto typeName = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(jsn, "_type"));
  const auto typeNameHash = hash(typeName);
  forEachComponentType([&]<typename T>(T *) {
    constexpr auto TName = getTypeName<T>();
    constexpr auto TNameHash = hash(TName);
    if (typeNameHash == TNameHash && typeName == TName) {
      init<T>(componentSnapshot);
      T &comp = *((T *)componentSnapshot.data);
      read(comp, jsn);
    }
  });
}

cJSON *write(const EditComponentSnapshot &componentSnapshot) {
  return componentSnapshot.write(componentSnapshot.data);
}


// Selection

static Seq<Entity> saveEditSelection() {
  Seq<Entity> result;
  each([&](Entity ent, EditSelect &sel) {
    result.push_back(ent);
  });
  return result;
}

static void loadEditSelection(const Seq<Entity> &selection) {
  each(remove<EditSelect>);
  for (auto ent : selection) {
    if (exists(ent)) {
      add<EditSelect>(ent);
    }
  }
}


// Reset

static void reset(EditSceneSnapshot &snapshot) {
  for (auto &entitySnapshot : snapshot.entitySnapshots()) {
    for (auto &componentSnapshot : entitySnapshot.componentSnapshots()) {
      componentSnapshot.free(componentSnapshot.data);
    }
  }
  snapshot = {};
}

static void resetEditHistory() {
  for (auto &snapshot : editHistory.snapshots()) {
    reset(snapshot);
  }
  editHistory = {};
}

static struct EditResetHistoryOnExit {
  ~EditResetHistoryOnExit() {
    resetEditHistory();
  }
} editResetHistoryOnExit;


// Save / load

void saveEditSnapshot(const char *desc, bool saveInspectedComponentTitle) {
  editHistory.lastIndex() = (editHistory.lastIndex() + 1) % editMaxSnapshots;

  auto &snapshot = editHistory.snapshots()[editHistory.lastIndex()];
  reset(snapshot);
  snapshot.valid() = true;
  snapshot.time() = rl::GetTime();
  copy(snapshot.description(), desc);
  if (saveInspectedComponentTitle) {
    copy(snapshot.inspectedComponentTitle(), edit.inspectedComponentTitle());
  }

  each([&](Entity ent) {
    if (!has<EditDelete>(ent)) {
      auto &entitySnapshot = snapshot.entitySnapshots().emplace_back();
      entitySnapshot.ent() = ent;
      forEachComponentType([&]<typename T>(T *) {
        if (has<T>(ent)) {
          auto &componentSnapshot = entitySnapshot.componentSnapshots().emplace_back();
          init<T>(componentSnapshot);
          T &destComp = *((T *)componentSnapshot.data);
          auto &srcComp = get<T>(ent);
          forEachProp(destComp, [&]<typename DestProp>(DestProp &destProp) {
            forEachProp(srcComp, [&]<typename SrcProp>(SrcProp &srcProp) {
              if constexpr (std::is_same_v<DestProp, SrcProp>) {
                destProp = srcProp;
              }
            });
          });
        }
      });
    }
  });
  snapshot.selection() = saveEditSelection();
}

static void loadEditSnapshot() {
  if (auto &snapshot = editHistory.snapshots()[editHistory.lastIndex()]; snapshot.valid()) {
    each(destroyEntity);
    for (auto &entitySnapshot : snapshot.entitySnapshots()) {
      auto ent = createEntity(entitySnapshot.ent());
      for (auto &componentSnapshot : entitySnapshot.componentSnapshots()) {
        componentSnapshot.add(componentSnapshot.data, ent);
      }
    }
  }
}


// Undo / redo

bool canUndoEdit() {
  auto &curr = editHistory.snapshots()[editHistory.lastIndex()];
  auto prevI = editHistory.lastIndex() == 0 ? editMaxSnapshots - 1 : editHistory.lastIndex() - 1;
  auto &prev = editHistory.snapshots()[prevI];
  return prev.valid() && (!curr.valid() || prev.time() <= curr.time());
}

void undoEdit() {
  auto &curr = editHistory.snapshots()[editHistory.lastIndex()];
  auto prevI = editHistory.lastIndex() == 0 ? editMaxSnapshots - 1 : editHistory.lastIndex() - 1;
  auto &prev = editHistory.snapshots()[prevI];
  if (prev.valid() && (!curr.valid() || prev.time() <= curr.time())) {
    editHistory.lastIndex() = prevI;
    loadEditSnapshot();
    if (curr.valid()) {
      loadEditSelection(curr.selection());
      if (!isEmpty(curr.inspectedComponentTitle())) {
        copy(edit.inspectedComponentTitle(), curr.inspectedComponentTitle());
      }
      notifyEdit("undid %s", curr.description());
    }
  }
}

bool canRedoEdit() {
  auto &curr = editHistory.snapshots()[editHistory.lastIndex()];
  auto nextI = editHistory.lastIndex() == editMaxSnapshots - 1 ? 0 : editHistory.lastIndex() + 1;
  auto &next = editHistory.snapshots()[nextI];
  return (next.valid() && (!curr.valid() || next.time() >= curr.time()));
}

void redoEdit() {
  auto &curr = editHistory.snapshots()[editHistory.lastIndex()];
  auto nextI = editHistory.lastIndex() == editMaxSnapshots - 1 ? 0 : editHistory.lastIndex() + 1;
  auto &next = editHistory.snapshots()[nextI];
  if (next.valid() && (!curr.valid() || next.time() >= curr.time())) {
    editHistory.lastIndex() = nextI;
    loadEditSnapshot();
    loadEditSelection(next.selection());
    if (!isEmpty(next.inspectedComponentTitle())) {
      copy(edit.inspectedComponentTitle(), next.inspectedComponentTitle());
    }
    notifyEdit("redid %s", next.description());
  }
}


//
// Session
//

struct EditSession {
  Prop(GameState, game);
  Prop(cJSON *, scene);
  Prop(EditState, edit);
  Prop(EditHistory, history);
  Prop(Seq<Entity>, selection);
};

JS_DEFINE(char *, JS_getStorage, (const char *key), {
  let found = localStorage.getItem(UTF8ToString(key));
  if (!found) {
    found = "";
  }
  return allocate(intArrayFromString(found), ALLOC_NORMAL);
});

JS_DEFINE(void, JS_setStorage, (const char *key, const char *value),
    { localStorage.setItem(UTF8ToString(key), UTF8ToString(value)); });

bool loadEditSession() {
  EditSession session;
  auto str = JS_getStorage("editSession");
  JS_setStorage("editSession", "");
  if (!str || isEmpty(str)) {
    std::free(str);
    return false;
  }
  auto jsn = cJSON_Parse(str);
  std::free(str);
  read(session, jsn);
  cJSON_Delete(jsn);

  each(destroyEntity);
  game = session.game();
  readScene(session.scene());
  cJSON_Delete(session.scene());

  edit = session.edit();

  resetEditHistory();
  editHistory = std::move(session.history());
  auto maxTime = -INFINITY;
  for (auto &snapshot : editHistory.snapshots()) {
    if (snapshot.valid() && snapshot.time() > maxTime) {
      maxTime = float(snapshot.time());
    }
  }
  for (auto &snapshot : editHistory.snapshots()) {
    snapshot.time() -= maxTime + 1;
  }

  loadEditSelection(session.selection());

  notifyEdit("restored session");
  return true;
}

JS_EXPORT void JS_saveEditSession() {
  EditSession session {
    .game { game },
    .scene { writeScene() },
    .edit { edit },
    .history { editHistory },
    .selection { saveEditSelection() },
  };
  auto jsn = write(session);
  cJSON_Delete(session.scene());
  auto str = stringify(jsn);
  JS_setStorage("editSession", str);
  cJSON_Delete(jsn);

  notifyEdit("saved session");
}


//
// Play / stop
//

void playEdit() {
  if (edit.enabled()) {
    editHistory.stopSelection() = saveEditSelection();
    edit.enabled() = false;
  }
}

void stopEdit() {
  if (!edit.enabled()) {
    edit.enabled() = true;
    loadEditSnapshot();
    loadEditSelection(editHistory.stopSelection());
  }
}
