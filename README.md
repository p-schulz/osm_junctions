# osm_junctions
Network generator with focus on gap-free lane and junction generation from OSM nodes.

## Building

```
cmake -S . -B build
cmake --build build -j
ctest --test-dir build
```

Two CLIs are built:

- `osm2xodr-procedural` (recommended): generates OpenDRIVE from OSM-derived control lines and
  control points -- deterministic, no random node/segment selection. See
  [docs/procedural_pipeline.md](docs/procedural_pipeline.md).
- `osm2xodr`: the original tag-inference pipeline (`src/model_builder.cpp`, `src/infer.cpp`). It is
  **stale** and superseded by `osm2xodr-procedural`; kept only for compatibility during migration
  and scheduled for removal. Do not extend it further.

Pass `-DOSM2XODR_ENABLE_LIBOPENDRIVE_VALIDATION=ON` to enable both CLIs' `--validate` flag (reads
generated `.xodr` back with the vendored libOpenDRIVE).

A GUI viewer, `osm2xodr-gui`, is also built by default (`-DOSM2XODR_BUILD_GUI=OFF` to skip it). It's
an [imgui](external/imgui)-based app (GLFW + OpenGL3 backend, fetched via CMake) with an "Import
OSM..." button in its command bar and a pan/zoom 2D preview of the resulting
`osm2xodr::procedural::GeneratedRoadGraph` -- roads, junction connectors, and control points colored
by type. Source: [src/gui/main.cpp](src/gui/main.cpp).

VSCode users: `.vscode/tasks.json` has Configure/Build/Test/Clean tasks for Debug and Release
(`Cmd+Shift+B` for the default Debug build), plus "Build GUI (Debug)" and "Run GUI (Debug)".
