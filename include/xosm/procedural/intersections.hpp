#pragma once

#include "xosm/procedural/config.hpp"
#include "xosm/procedural/types.hpp"

namespace xosm::procedural {

// For every control point with incident-connection degree >= config.junction_min_degree:
//  - trims each incident Connection's geometry back from the control point by a lane-width- and
//    turn-radius-derived setback so a junction interior exists to route through;
//  - computes each leg's heading at the trim point and classifies candidate movements as
//    through/left/right (config.left_hand_traffic aware), skipping U-turns;
//  - for each movement whose incoming leg has turn:lanes-eligible incoming lanes (or is
//    unrestricted) and whose outgoing leg has outgoing lanes, generates one synthetic connecting
//    Connection with a ParamPoly3 (ctrl_p1/2/3 -- cubic Bezier, model.hpp's own primitive) interior
//    curve, lane-linked to the paired incoming/outgoing lanes (config.split_strategy governs how a
//    lane-count mismatch within one movement is folded);
//  - guarantees every incoming lane ends up with at least one outgoing connection (relaxing an
//    unsatisfiable turn:lanes restriction with a recorded warning rather than leaving a dead lane);
//  - appends one model::Junction (junction/laneLink metadata, via the existing model.hpp/xodr_writer
//    contract) and records the control point -> junction id mapping so model_assembly.cpp links the
//    incident (now-trimmed) roads' own predecessor/successor to the junction instead of each other.
void generate_intersections(GeneratedRoadGraph& graph, const GeneratorConfig& config);

// Deterministic turn radius (meters) for a highway=* class, driving both a junction connector's
// Bezier tangent-handle length and its incident roads' setback (see generate_intersections).
// Resolution order: an exact match in config.junction_turn_radius_overrides, else a built-in
// per-class default tier (motorway/trunk widest, residential/service tightest), else
// config.junction_turn_radius_m as the final fallback -- the result is always multiplied by
// config.junction_turn_radius_scale.
double turn_radius_for_highway(const std::string& highway, const GeneratorConfig& config);

} // namespace xosm::procedural
