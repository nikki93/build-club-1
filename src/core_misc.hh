#pragma once
#include "core.hh"


//
// Strings
//

template<int N>
void copy(char (&dest)[N], const char *src) {
  if constexpr (N > 0) {
    dest[0] = '\0';
    std::strncat(dest, src, N - 1);
  }
}

template<int N>
const char *bprint(char (&buf)[N], const char *format, auto &&...args) {
  std::snprintf(buf, N, format, std::forward<decltype(args)>(args)...);
  return buf;
}

template<int N>
const char *bprint(char (&buf)[N], const char *str) {
  copy(buf, str);
  return buf;
}

template<int N>
const char *bprint(const char *format, auto &&...args) {
  static char buf[N];
  return bprint(buf, format, std::forward<decltype(args)>(args)...);
}

template<int N>
const char *nullTerminate(std::string_view str) {
  static char buf[N];
  buf[0] = '\0';
  std::strncat(buf, str.data(), std::min(N - 1, int(str.size())));
  buf[str.size()] = '\0';
  return buf;
}

inline bool isEmpty(const char *str) {
  return str[0] == '\0';
}


//
// Console
//

inline void cprint(const char *format, auto &&...args) {
  std::printf(format, std::forward<decltype(args)>(args)...);
  std::puts("");
}

inline void cprint(const char *msg) {
  std::puts(msg);
}


//
// Debug display
//

inline char debugDisplayBuffer[1024] = "";
inline char *debugDisplayCursor = debugDisplayBuffer;

inline void clearDebugDisplay() {
  debugDisplayBuffer[0] = '\0';
  debugDisplayCursor = debugDisplayBuffer;
}

inline void dprint(const char *format, auto &&...args) {
  auto remaining = sizeof(debugDisplayBuffer) - (debugDisplayCursor - debugDisplayBuffer);
  if (remaining > 0) {
    debugDisplayCursor += std::snprintf(
        debugDisplayCursor, remaining, format, std::forward<decltype(args)>(args)...);
    if (sizeof(debugDisplayBuffer) - (debugDisplayCursor - debugDisplayBuffer) > 2) {
      *debugDisplayCursor++ = '\n';
      *debugDisplayCursor = '\0';
    }
  }
}

inline void dprint(const char *msg) {
  dprint("%s", msg);
}
