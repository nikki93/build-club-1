# build-club-1

This is the starter kit code referenced in [my talk for Build Club](https://www.youtube.com/watch?v=0FjCaoc1uzs).

## Build instructions

Make sure to have checked out the repository along with submodules -- so either
`git clone --recursive https://github.com/nikki93/build-club-1.git` at first,
or `git submodule update --init --recursive` after cloning.

You will need CMake and a way to run bash scripts for all platforms (so -- WSL
on Windows -- maybe I should add a batch file or something though...).

### Web

Just run `./run.sh web-release` to do a web build in release mode. This will
take quite a bit the first time to download some dependencies (all happens in
the 'vendor/emsdk' subdirectory -- nothing outside this repo) and do the first build.

Then, `./run.sh web-serve-release` (needs node -- any other way to serve the
'./build/web-release' directory with a static web file server will work
though). You should see the project running at
[http://localhost:9002/index.html](http://localhost:9002/index.html).

After that, run `./run.sh web-release` again whenever you make changes to build
again. Subsequent builds should be quicker.

### Desktop

`./run.sh release` will build and run for desktop.
