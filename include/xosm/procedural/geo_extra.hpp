#pragma once

#include "xosm/geo.hpp"
#include "xosm/model.hpp"

#include <optional>
#include <vector>

// Small geometry helpers that extend xosm::geo (geo.hpp) for the procedural generator, without
// modifying that existing coordinate/projection utility module.
namespace xosm::procedural::geo_extra {

// Ramer-Douglas-Peucker polyline simplification. Endpoints are always preserved; `tolerance_m` is
// the max perpendicular deviation a dropped vertex may introduce. Used to turn a merged OSM
// corridor's raw vertex chain into a control line's reference geometry while preserving real
// curvature (a large tolerance degenerates to a straight line; a small one keeps most vertices).
// `must_keep_indices` are additional original-array indices that always survive simplification
// exactly (e.g. a real OSM junction node embedded in the interior of a merged corridor) -- passed
// as indices into `points`, not required to be sorted or unique.
std::vector<geo::Vec2> douglas_peucker(const std::vector<geo::Vec2>& points, double tolerance_m,
                                        const std::vector<std::size_t>& must_keep_indices = {});

// Inverse of geo::LocalProjector::project (equirectangular, flat-earth approximation valid at the
// projector's own local scale) -- needed only for emitting human-readable WGS84 coordinates in debug
// GeoJSON output; the pipeline itself stays entirely in the projected Vec2 frame.
geo::LonLat unproject(const geo::LocalProjector& projector, const geo::Vec2& p);

// True if closed segments [a0,a1] and [b0,b1] cross at an interior point of both (endpoint touches
// don't count -- those are shared-node topology, handled separately). `out` receives the crossing
// point when true.
bool segments_cross(const geo::Vec2& a0, const geo::Vec2& a1, const geo::Vec2& b0, const geo::Vec2& b1,
                     geo::Vec2* out);

double polyline_heading_near(const std::vector<geo::Vec2>& points, bool near_start, double sample_len_m);

// Removes `dist` meters of arclength from the specified end of `pts` (clamped so at least 1m of
// road remains), mutating `pts` in place. Returns the new endpoint. Used to carve out room for a
// junction/bridge interior before generating its connecting geometry.
geo::Vec2 trim_polyline_end(std::vector<geo::Vec2>& pts, bool at_end, double dist);

// Builds a single ParamPoly3 (cubic Bezier control points, model.hpp's own primitive) connecting
// `p0` (with tangent direction `hdg0`, pointing into the curve) to `p3` (with tangent direction
// `hdg3`, pointing along the direction of travel leaving the curve) via the standard Hermite-to-
// Bezier construction, with an approximate arclength `length` (the usual
// (chord + control-polygon length)/2 cubic-Bezier estimate).
//
// `turn_radius_m`, when given (and positive), drives the tangent-handle length from a real turn
// radius R instead of a flat chord fraction: `k = R * 4/3 * tan(theta/4)`, the standard cubic-Bezier
// approximation of a circular arc of radius R subtending deflection angle theta (the angle between
// hdg0 extended and hdg3). A floor of `max(chord*0.1, 0.5)` keeps near-straight movements (theta ~
// 0, where the radius formula alone collapses toward a zero-length, kinked handle) from looking
// pinched. Omitted (the default): unchanged flat `chord/3` handle -- used by cleanup.cpp's lane-
// count-mismatch bridges, which aren't turning movements and have no radius to speak of.
model::GeomPrimitive hermite_bezier_geometry(const geo::Vec2& p0, double hdg0, const geo::Vec2& p3, double hdg3,
                                              std::optional<double> turn_radius_m = std::nullopt);

// Maps `count_in` lane slots onto `count_out` lane slots (both center-outward local indices) as a
// set of (in,out) pairs covering every slot on both sides at least once, used both for a junction
// movement's lane connections (intersections.cpp) and a plain lane-count-mismatch bridge
// (cleanup.cpp). `split_first` picks which physical lane absorbs a count mismatch: the innermost
// (center-side) lane when true, the outermost when false.
std::vector<std::pair<int, int>> pair_lane_slots(int count_in, int count_out, bool split_first);

} // namespace xosm::procedural::geo_extra
