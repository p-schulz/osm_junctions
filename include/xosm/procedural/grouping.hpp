#pragma once

#include "xosm/procedural/config.hpp"
#include "xosm/procedural/types.hpp"

#include <vector>

namespace xosm::procedural {

// Groups control lines by OSM topology instead of JunctionArt's random pairing:
//  - Component: control lines connected (directly or transitively) via a shared control point.
//  - Crossing: control lines that share one specific OsmIntersection/ProjectedCrossing control
//    point -- the lines bounding that junction.
//  - Parallel: same-component control lines running roughly alongside each other (similar heading,
//    small perpendicular offset over an overlapping span) -- metadata only; nothing in the default
//    pipeline fabricates a road between a Parallel pair (see cleanup.hpp's repair pass for the one
//    place synthetic connectors are ever created, and only for boundary/mismatch repair).
std::vector<ControlLineGroup> group_control_lines(const std::vector<ControlLine>& control_lines,
                                                    const std::vector<ControlPoint>& control_points,
                                                    const GeneratorConfig& config);

} // namespace xosm::procedural
