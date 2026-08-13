#pragma once

#include "osm2xodr/procedural/config.hpp"
#include "osm2xodr/procedural/types.hpp"

namespace osm2xodr::procedural {

// For every control point with incident-connection degree >= config.junction_min_degree:
//  - trims each incident Connection's geometry back from the control point by a lane-width-derived
//    setback so a junction interior exists to route through;
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

} // namespace osm2xodr::procedural
