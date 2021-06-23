#pragma once
#include "core.hh"


inline void read(auto &val, const cJSON *jsn); // Forward declaration
inline cJSON *write(const auto &val);


//
// Assets
//

inline const char *getAssetPath(const char *assetName) {
  return bprint<128>("assets/%s", assetName);
}

inline const char *getAssetContents(const char *assetName) {
  static char buf[500 * 1024];
  auto file = std::fopen(getAssetPath(assetName), "r");
  std::fseek(file, 0, SEEK_END);
  auto size = std::ftell(file);
  if (long(sizeof(buf)) < size + 1) {
    cprint("file '%s' is too large to read into memory!\n", assetName);
    std::fclose(file);
    return "";
  }
  std::rewind(file);
  std::fread(buf, 1, size, file);
  std::fclose(file);
  buf[size] = '\0';
  return buf;
}

inline void writeAssetContents(const char *assetName, const char *contents) {
  auto file = std::fopen(getAssetPath(assetName), "w");
  std::fputs(contents, file);
  std::fclose(file);
}


//
// Primitives
//

// int
inline void read(int &val, const cJSON *jsn) {
  if (cJSON_IsNumber(jsn)) {
    val = jsn->valueint;
  }
}
inline cJSON *write(const int &val) {
  return cJSON_CreateNumber(val);
}

// float
inline void read(float &val, const cJSON *jsn) {
  if (cJSON_IsNumber(jsn)) {
    val = float(jsn->valuedouble);
  }
}
inline cJSON *write(const float &val) {
  return cJSON_CreateNumber(val);
}

// double
inline void read(double &val, const cJSON *jsn) {
  if (cJSON_IsNumber(jsn)) {
    val = jsn->valuedouble;
  }
}
inline cJSON *write(const double &val) {
  return cJSON_CreateNumber(val);
}

// bool
inline void read(bool &val, const cJSON *jsn) {
  if (cJSON_IsBool(jsn)) {
    val = cJSON_IsTrue(jsn);
  }
}
inline cJSON *write(const bool &val) {
  return cJSON_CreateBool(val);
}

// char array
template<int N>
void read(char (&val)[N], const cJSON *jsn) {
  if (cJSON_IsString(jsn)) {
    copy(val, cJSON_GetStringValue(jsn));
  }
}
inline cJSON *write(const char *val) {
  return cJSON_CreateString(val);
}

// Entity
inline void read(Entity &val, const cJSON *jsn) {
  if (cJSON_IsNumber(jsn)) {
    val = Entity(jsn->valueint);
  }
}
inline cJSON *write(const Entity &val) {
  return cJSON_CreateNumber(uint64_t(val));
}


//
// Graphics
//

// PERF: Can use direct access to nodes -- array get scans

// Vec2
inline void read(Vec2 &val, const cJSON *jsn) {
  if (cJSON_IsArray(jsn) && cJSON_GetArraySize(jsn) == 2) {
    read(val.x, cJSON_GetArrayItem(jsn, 0));
    read(val.y, cJSON_GetArrayItem(jsn, 1));
  }
}
inline cJSON *write(const Vec2 &val) {
  auto result = cJSON_CreateArray();
  cJSON_AddItemToArray(result, cJSON_CreateNumber(val.x));
  cJSON_AddItemToArray(result, cJSON_CreateNumber(val.y));
  return result;
}

// rl::Rectangle
inline void read(rl::Rectangle &val, const cJSON *jsn) {
  if (cJSON_IsArray(jsn) && cJSON_GetArraySize(jsn) == 4) {
    read(val.x, cJSON_GetArrayItem(jsn, 0));
    read(val.y, cJSON_GetArrayItem(jsn, 1));
    read(val.width, cJSON_GetArrayItem(jsn, 2));
    read(val.height, cJSON_GetArrayItem(jsn, 3));
  }
}
inline cJSON *write(const rl::Rectangle &val) {
  auto result = cJSON_CreateArray();
  cJSON_AddItemToArray(result, cJSON_CreateNumber(val.x));
  cJSON_AddItemToArray(result, cJSON_CreateNumber(val.y));
  cJSON_AddItemToArray(result, cJSON_CreateNumber(val.width));
  cJSON_AddItemToArray(result, cJSON_CreateNumber(val.height));
  return result;
}

// rl::Camera
struct ReadWriteCamera {
  Prop(Vec2, offset) { 0, 0 };
  Prop(Vec2, target) { 0, 0 };
  Prop(float, rotation) { 0 };
  Prop(float, zoom) { 1 };
};
inline void read(rl::Camera2D &val, const cJSON *jsn) {
  ReadWriteCamera rw { { val.offset }, { val.target }, { val.rotation }, { val.zoom } };
  read(rw, jsn);
  val = { rw.offset(), rw.target(), rw.rotation(), rw.zoom() };
}
inline cJSON *write(const rl::Camera2D &val) {
  return write(ReadWriteCamera { { val.offset }, { val.target }, { val.rotation }, { val.zoom } });
}


//
// Containers
//

// T[N]
template<typename T, unsigned N>
void read(T (&val)[N], const cJSON *jsn) {
  if (cJSON_IsArray(jsn)) {
    auto i = 0;
    for (auto elemJsn = jsn->child; elemJsn; elemJsn = elemJsn->next) {
      if (i >= int(N)) {
        break;
      }
      read(val[i++], elemJsn);
    }
  }
}
template<typename T, unsigned N>
cJSON *write(const T (&val)[N]) {
  auto result = cJSON_CreateArray();
  for (auto i = 0; i < int(N); ++i) {
    cJSON_AddItemToArray(result, write(val[i]));
  }
  return result;
}

// Seq<T, N>
template<typename T, unsigned N>
void read(Seq<T, N> &val, const cJSON *jsn) {
  if (cJSON_IsArray(jsn)) {
    val.clear();
    val.reserve(cJSON_GetArraySize(jsn));
    for (auto elemJsn = jsn->child; elemJsn; elemJsn = elemJsn->next) {
      read(val.emplace_back(), elemJsn);
    }
  }
}
template<typename T, unsigned N>
cJSON *write(const Seq<T, N> &val) {
  auto result = cJSON_CreateArray();
  for (auto &elem : val) {
    cJSON_AddItemToArray(result, write(elem));
  }
  return result;
}

// cJSON *
inline void read(cJSON *&val, const cJSON *jsn) {
  val = cJSON_Duplicate(jsn, true);
}
inline cJSON *write(cJSON *const &jsn) {
  return cJSON_Duplicate(jsn, true);
}


//
// Props
//

void read(auto &val, const cJSON *jsn) {
  if (cJSON_IsObject(jsn)) {
    for (auto elemJsn = jsn->child; elemJsn; elemJsn = elemJsn->next) {
      const auto key = elemJsn->string;
      const auto keyHash = hash(key);
      forEachProp(val, [&]<typename P>(P &prop) {
        if (keyHash == P::nameHash && key == P::name) {
          read(prop(), elemJsn);
        }
      });
    }
  }
}
cJSON *write(const auto &val) {
  auto result = cJSON_CreateObject();
  forEachProp(val, [&]<typename P>(P &prop) {
    cJSON_AddItemToObjectCS(result, P::name.data(), write(prop()));
  });
  return result;
}


//
// `cJSON *` -> `const char *`
//

inline const char *stringify(cJSON *jsn, bool formatted = false) {
  static char buf[500 * 1024];
  if (!cJSON_PrintPreallocated(jsn, buf, sizeof(buf), formatted)) {
    cprint("json is too large to stringify!");
    return "";
  }
  return buf;
}


//
// Scene
//

void readScene(const cJSON *jsn);
void readScene(const char *assetName);

cJSON *writeScene();
const char *writeSceneToString(bool formatted = false);
void writeSceneToAsset(const char *assetName, bool formatted = false);
