#pragma once

// C standard library
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cctype>

// Emscripten
#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#endif

// raylib
namespace rl {
#include "raylib.h"
#include "rlgl.h"
}

// entt
#include <entt/entity/registry.hpp>

// cJSON
#include <cJSON.h>

// Chipmunk2D
namespace cp {
#include <chipmunk/chipmunk.h>
#include <chipmunk/chipmunk_private.h>
#include <chipmunk/chipmunk_unsafe.h>
}

// SmallVector
#include "../vendor/small_vector.h"
template<typename T, unsigned N = 0>
using Seq = smallvec::SmallVector<T, N>;

// JS import / export
#ifdef __EMSCRIPTEN__
#define JS_DEFINE(retType, name, ...) EM_JS(retType, name, __VA_ARGS__)
#else
#define JS_DEFINE(retType, name, args, ...)                                                        \
  using name##RetType = retType;                                                                   \
  retType name args {                                                                              \
    return name##RetType();                                                                        \
  }
#endif
#ifdef __EMSCRIPTEN__
#define JS_EXPORT EMSCRIPTEN_KEEPALIVE extern "C"
#else
#define JS_EXPORT
#endif
