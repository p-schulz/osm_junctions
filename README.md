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
