# XOSM

Generates OpenDRIVE (`.xodr`) road networks from OpenStreetMap data, using control lines and
control points derived deterministically from real OSM geometry and topology -- no random node or
segment selection. See [docs/procedural_pipeline.md](docs/procedural_pipeline.md) for the pipeline.

## Building

```
cmake -S . -B build
cmake --build build -j
ctest --test-dir build
```

This builds:

- **`xosm`** -- the CLI. `xosm <input.osm|osm.pbf> <output.xodr> [options]`; see `xosm --help`.
- **`xosm-gui`** -- an [imgui](external/imgui)-based viewer (GLFW + OpenGL3 backend, fetched via
  CMake; `-DXOSM_BUILD_GUI=OFF` to skip it). Its command bar imports an OSM file and exports the
  generated network to `.xodr`; the main area is a pan/zoom 2D preview of the generated
  `xosm::procedural::GeneratedRoadGraph` rendered lane-by-lane (each lane's own width/side), with
  junction connectors and control points colored by type. Source: [src/gui/main.cpp](src/gui/main.cpp).

Pass `-DXOSM_ENABLE_LIBOPENDRIVE_VALIDATION=ON` to enable `xosm`'s `--validate` flag (reads the
generated `.xodr` back with the vendored libOpenDRIVE).

VSCode users: `.vscode/tasks.json` has Configure/Build/Test/Clean tasks for Debug and Release
(`Cmd+Shift+B` for the default Debug build), plus "Build GUI (Debug)" and "Run GUI (Debug)".
