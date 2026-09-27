# tww_setupfinder

A desktop app that finds movement setups for The Legend of Zelda: The Wind Waker. You choose a
room, where Link starts and where he needs to be, and it searches for inputs that get him there.
Every setup it returns is checked frame by frame against the game's own physics.

## What you need

- **A Japanese Wind Waker disc image (GZLJ01).** You can't search without it: Link's model and
  animations come from it, and so do the rooms unless a running game supplies one. Nothing from
  the game ships with this app.
- **Dolphin**, if you want to read the room and Link's position from a running game. It is
  optional.

## Reading a running game

| | |
|---|---|
| Windows | Works. |
| Linux | Works once one setting is changed (below). |
| macOS | Not supported. Everything else works. |

On Linux, if `Connect` does nothing, most distributions stop one program reading another. Allow it
for the session:

    sudo sysctl -w kernel.yama.ptrace_scope=0

or start the app from the same shell that starts Dolphin.

Only the Japanese release can be read. Other versions show in the `Emulators` window but cannot be
chosen.

## Building from source

You need Node.js 20 or newer, Rust (stable), CMake 3.20 or newer and a C++17 compiler. On Windows
that is Visual Studio 2022 with the C++ workload; the WebView2 runtime ships with Windows 11. On
Linux, Tauri also needs its [system packages](https://tauri.app/start/prerequisites/).

    git clone --recursive https://github.com/DMiller0090/tww_setupfinder.git
    cd tww_setupfinder
    node build.mjs

Tests that need the disc skip without `SETUPCORE_ISO`.

| Command | What it does |
|---|---|
| `node build.mjs` | Builds the core and the frontend, then runs the suites |
| `node build.mjs test` | Runs the suites against what is already built |
| `node build.mjs run` | Builds, then opens the app |
| `node build.mjs release` | Builds, then makes the zip for this OS under `app/release/` |
| `--stop` | Closes a running app first, since it holds the core open |
| `--debug` | Builds the core in Debug |

## Layout

| Folder | What it is |
|---|---|
| `app/` | The frontend: Svelte and TypeScript |
| `app/src-tauri/` | The Tauri shell, which starts the core and passes messages to it |
| `core/` | The C++ backend: the search, the disc reader and the Dolphin reader |
| `external/tww_engine/` | The physics library the search checks every setup with |

## License

MIT; see [LICENSE](LICENSE). The physics come from
[tww_engine](https://github.com/DMiller0090/tww_engine), which is derived from the
[zeldaret/tww](https://github.com/zeldaret/tww) decompilation (CC0); see [NOTICE](NOTICE).
