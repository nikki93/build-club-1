#!/bin/bash

export PROJECT_NAME="build-club-1"
export WSLENV="PROJECT_NAME"

set -e

PLATFORM="macOS"
CMAKE="cmake"
CLANG_FORMAT="clang-format"
TIME="time"
TIME_TOTAL="time"

if [[ -f /proc/version ]]; then
  if grep -q Linux /proc/version; then
    PLATFORM="lin"
    TIME="time --format=%es\n"
    TIME_TOTAL="time --format=total\t%es\n"
  fi
  if grep -q Microsoft /proc/version; then
    PLATFORM="win"
    CMAKE="cmake.exe"
    CLANG_FORMAT="clang-format.exe"
  fi
fi
CMAKE="$TIME $CMAKE"

case "$1" in
  # Compile commands DB (used by editor plugins)
  db)
    $CMAKE -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug -H. -Bbuild/db -GNinja
    cp ./build/db/compile_commands.json .
    ;;

  # Format
  format)
    $CLANG_FORMAT -i -style=file $(find src/ -type f)
    ;;

  # Count lines of code
  cloc)
    cloc src --by-file --exclude_list_file=.cloc_exclude_list
    ;;

  # Desktop
  release)
    $CMAKE -H. -Bbuild/release -GNinja
    $CMAKE --build build/release
    case $PLATFORM in
      lin|macOS)
        ./build/release/$PROJECT_NAME $2
        ;;
      win)
        ./build/release/$PROJECT_NAME.exe $2
        ;;
    esac
    ;;
  debug)
    $CMAKE -DCMAKE_BUILD_TYPE=Debug -H. -Bbuild/debug -GNinja
    $CMAKE --build build/debug
    case $PLATFORM in
      lin|macOS)
        ./build/debug/$PROJECT_NAME $2
        ;;
      win)
        ./build/debug/$PROJECT_NAME.exe $2
        ;;
    esac
    ;;

  # Web
  web-init)
    case $PLATFORM in
      lin|macOS)
        cd vendor/emsdk
        ./emsdk install latest
        ./emsdk activate latest
        ;;
      win)
        cd vendor/emsdk
        cmd.exe /c emsdk install latest
        cmd.exe /c emsdk activate latest
        ;;
    esac
    ;;
  web-release)
    if [[ ! -f "vendor/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake" ]]; then
      ./run.sh web-init
    fi
    if [[ ! -d "build/web-release" ]]; then
      $CMAKE -DWEB=ON -H. -Bbuild/web-release -GNinja
    fi
    $CMAKE --build build/web-release
    touch build/web-release/reload-trigger
    ;;
  web-debug)
    if [[ ! -f "vendor/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake" ]]; then
      ./run.sh web-init
    fi
    $CMAKE --build build/web-debug
    touch build/web-release/reload-trigger
    ;;
  web-watch-release)
    find CMakeLists.txt src assets web -type f | entr $TIME_TOTAL ./run.sh web-release
    ;;
  web-watch-debug)
    find CMakeLists.txt src assets web -type f | entr $TIME_TOTAL ./run.sh web-debug
    ;;
  web-serve-release)
    npx http-server -p 9002 -c-1 build/web-release
    ;;
  web-serve-debug)
    npx http-server -p 9002 -c-1 build/web-debug
    ;;
esac
