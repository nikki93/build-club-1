#include "core.hh"

#include "game.hh"
UseComponentTypes();


//
// Scene
//

void readScene(const cJSON *jsn) {
  for (auto entJsn = cJSON_GetObjectItemCaseSensitive(jsn, "entities")->child; entJsn;
       entJsn = entJsn->next) {
    Entity ent;
    if (auto idJsn = cJSON_GetObjectItemCaseSensitive(entJsn, "id")) {
      ent = createEntity(Entity(cJSON_GetNumberValue(idJsn)));
    } else {
      ent = createEntity();
    }
    for (auto compJsn = cJSON_GetObjectItemCaseSensitive(entJsn, "components")->child; compJsn;
         compJsn = compJsn->next) {
      const auto typeName
          = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(compJsn, "_type"));
      const auto typeNameHash = hash(typeName);
      forEachComponentType([&]<typename T>(T *) {
        constexpr auto TName = getTypeName<T>();
        constexpr auto TNameHash = hash(TName);
        if (typeNameHash == TNameHash && typeName == TName) {
          T comp {};
          read(comp, compJsn);
          add<T>(ent, std::move(comp));
        }
      });
    }
  }
}

void readScene(const char *assetName) {
  auto root = cJSON_Parse(getAssetContents(assetName));
  readScene(root);
  cJSON_Delete(root);
}


cJSON *writeScene() {
  auto result = cJSON_CreateObject();

  Seq<Entity> entities;
  each([&](Entity ent) {
    entities.push_back(ent);
  });
  std::reverse(entities.begin(), entities.end());
  auto entitiesJsn = cJSON_CreateArray();
  cJSON_AddItemToObjectCS(result, "entities", entitiesJsn);
  for (auto ent : entities) {
    auto entityJsn = cJSON_CreateObject();
    cJSON_AddItemToArray(entitiesJsn, entityJsn);

    auto idJsn = cJSON_CreateNumber(double(uint64_t(ent)));
    cJSON_AddItemToObjectCS(entityJsn, "id", idJsn);

    auto compsJsn = cJSON_CreateArray();
    cJSON_AddItemToObjectCS(entityJsn, "components", compsJsn);
    forEachComponentType([&]<typename T>(T *) {
      if (has<T>(ent)) {
        auto &comp = get<T>(ent);
        auto compJsn = write(comp);

        cJSON_AddItemToObjectCS(
            compJsn, "_type", cJSON_CreateString(nullTerminate<64>(getTypeName<T>())));
        if (compJsn->child->prev != compJsn->child) {
          compJsn->child->prev->next = compJsn->child;
          compJsn->child = compJsn->child->prev;
          compJsn->child->prev->next = NULL;
        }

        cJSON_AddItemToArray(compsJsn, compJsn);
      }
    });
  }
  return result;
}

const char *writeSceneToString(bool formatted) {
  auto jsn = writeScene();
  auto result = stringify(jsn, formatted);
  cJSON_Delete(jsn);
  return result;
}

void writeSceneToAsset(const char *assetName, bool formatted) {
  auto contents = writeSceneToString(formatted);
  if (!isEmpty(contents)) {
    writeAssetContents(assetName, contents);
  }
}
