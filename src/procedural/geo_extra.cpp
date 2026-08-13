#include "xosm/procedural/geo_extra.hpp"

#include <algorithm>
#include <cmath>

namespace xosm::procedural::geo_extra {

namespace {

void dp_recurse(const std::vector<geo::Vec2>& pts, const std::size_t lo, const std::size_t hi,
                 const double tolerance_m, std::vector<bool>& keep) {
    if (hi <= lo + 1) return;
    const geo::Vec2& a = pts[lo];
    const geo::Vec2& b = pts[hi];
    const geo::Vec2 ab = b - a;
    const double ab_len2 = geo::dot(ab, ab);

    double best_dist = -1.0;
    std::size_t best_idx = lo;
    for (std::size_t i = lo + 1; i < hi; ++i) {
        double dist;
        if (ab_len2 <= 1e-12) {
            dist = geo::length(pts[i] - a);
        } else {
            double u = geo::dot(pts[i] - a, ab) / ab_len2;
            u = std::max(0.0, std::min(1.0, u));
            const geo::Vec2 q = a + ab * u;
            dist = geo::length(pts[i] - q);
        }
        if (dist > best_dist) {
            best_dist = dist;
            best_idx = i;
        }
    }

    if (best_dist > tolerance_m) {
        keep[best_idx] = true;
        dp_recurse(pts, lo, best_idx, tolerance_m, keep);
        dp_recurse(pts, best_idx, hi, tolerance_m, keep);
    }
}

} // namespace

std::vector<geo::Vec2> douglas_peucker(const std::vector<geo::Vec2>& points, const double tolerance_m,
                                        const std::vector<std::size_t>& must_keep_indices) {
    if (points.size() <= 2) return points;
    std::vector<bool> keep(points.size(), false);
    keep.front() = true;
    keep.back() = true;
    for (const auto idx : must_keep_indices)
        if (idx < keep.size()) keep[idx] = true;

    std::vector<std::size_t> boundaries;
    for (std::size_t i = 0; i < keep.size(); ++i)
        if (keep[i]) boundaries.push_back(i);
    for (std::size_t b = 1; b < boundaries.size(); ++b)
        dp_recurse(points, boundaries[b - 1], boundaries[b], tolerance_m, keep);

    std::vector<geo::Vec2> out;
    out.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i)
        if (keep[i]) out.push_back(points[i]);
    return out;
}

geo::LonLat unproject(const geo::LocalProjector& projector, const geo::Vec2& p) {
    geo::LonLat ll;
    ll.lat = p.y / geo::kEarthRadiusM * 180.0 / geo::kPi + projector.origin.lat;
    const double cos_lat0 = projector.cos_lat0 <= 1e-9 ? 1.0 : projector.cos_lat0;
    ll.lon = p.x / (geo::kEarthRadiusM * cos_lat0) * 180.0 / geo::kPi + projector.origin.lon;
    return ll;
}

bool segments_cross(const geo::Vec2& a0, const geo::Vec2& a1, const geo::Vec2& b0, const geo::Vec2& b1,
                     geo::Vec2* out) {
    const auto tu = geo::line_intersect_params(a0, a1 - a0, b0, b1 - b0);
    if (!tu) return false;
    const auto [t, u] = *tu;
    constexpr double kEps = 1e-6;
    if (t <= kEps || t >= 1.0 - kEps || u <= kEps || u >= 1.0 - kEps) return false;
    if (out) *out = a0 + (a1 - a0) * t;
    return true;
}

geo::Vec2 trim_polyline_end(std::vector<geo::Vec2>& pts, const bool at_end, double dist) {
    if (pts.size() < 2) return pts.empty() ? geo::Vec2{} : pts.front();
    const double total = geo::polyline_length(pts);
    dist = std::min(dist, std::max(0.0, total - 1.0));
    if (!at_end) std::reverse(pts.begin(), pts.end());
    double remaining = dist;
    while (pts.size() > 1 && remaining > 1e-9) {
        const double seg = geo::length(pts.back() - pts[pts.size() - 2]);
        if (seg > remaining) {
            pts.back() = pts.back() + (pts[pts.size() - 2] - pts.back()) * (remaining / seg);
            remaining = 0.0;
        } else {
            remaining -= seg;
            pts.pop_back();
        }
    }
    const geo::Vec2 result = pts.back();
    if (!at_end) std::reverse(pts.begin(), pts.end());
    return result;
}

model::GeomPrimitive hermite_bezier_geometry(const geo::Vec2& p0, const double hdg0, const geo::Vec2& p3, const double hdg3) {
    const double chord = geo::length(p3 - p0);
    const double lever = std::max(chord / 3.0, 0.5);
    const geo::Vec2 t0{std::cos(hdg0), std::sin(hdg0)};
    const geo::Vec2 t3{std::cos(hdg3), std::sin(hdg3)};
    const geo::Vec2 p1 = p0 + t0 * lever;
    const geo::Vec2 p2 = p3 - t3 * lever;

    const auto to_local = [&](const geo::Vec2& p) {
        const geo::Vec2 d = p - p0;
        const double c = std::cos(-hdg0), s = std::sin(-hdg0);
        return geo::Vec2{d.x * c - d.y * s, d.x * s + d.y * c};
    };

    model::GeomPrimitive g;
    g.x = p0.x;
    g.y = p0.y;
    g.hdg = hdg0;
    g.kind = model::GeomKind::ParamPoly3;
    g.local_p1 = to_local(p1);
    g.local_p2 = to_local(p2);
    g.local_p3 = to_local(p3);
    const double poly = geo::length(p1 - p0) + geo::length(p2 - p1) + geo::length(p3 - p2);
    g.length = std::max(0.5, (chord + poly) / 2.0);
    return g;
}

std::vector<std::pair<int, int>> pair_lane_slots(const int count_in, const int count_out, const bool split_first) {
    std::vector<std::pair<int, int>> pairs;
    if (count_in <= 0 || count_out <= 0) return pairs;
    const int extra = count_out - count_in;
    if (extra >= 0) { // split: every outgoing lane covered; `extra` beyond a straight 1:1 mapping
        if (split_first) {
            for (int o = 0; o <= extra; ++o) pairs.emplace_back(0, o);
            for (int i = 1; i < count_in; ++i) pairs.emplace_back(i, i + extra);
        } else {
            for (int i = 0; i < count_in - 1; ++i) pairs.emplace_back(i, i);
            for (int o = count_in - 1; o < count_out; ++o) pairs.emplace_back(count_in - 1, o);
        }
    } else { // merge: every incoming lane covered; `-extra` beyond a straight 1:1 mapping
        const int deficit = -extra;
        if (split_first) {
            for (int i = 0; i <= deficit; ++i) pairs.emplace_back(i, 0);
            for (int o = 1; o < count_out; ++o) pairs.emplace_back(o + deficit, o);
        } else {
            for (int o = 0; o < count_out - 1; ++o) pairs.emplace_back(o, o);
            for (int i = count_out - 1; i < count_in; ++i) pairs.emplace_back(i, count_out - 1);
        }
    }
    return pairs;
}

double polyline_heading_near(const std::vector<geo::Vec2>& points, const bool near_start, const double sample_len_m) {
    if (points.size() < 2) return 0.0;
    if (near_start) {
        double acc = 0.0;
        for (std::size_t i = 1; i < points.size(); ++i) {
            const double seg = geo::length(points[i] - points[i - 1]);
            acc += seg;
            if (acc >= sample_len_m || i + 1 == points.size()) return geo::heading(points[0], points[i]);
        }
        return geo::heading(points.front(), points.back());
    }
    double acc = 0.0;
    for (std::size_t i = points.size() - 1; i > 0; --i) {
        const double seg = geo::length(points[i] - points[i - 1]);
        acc += seg;
        if (acc >= sample_len_m || i == 1) return geo::heading(points[i - 1], points.back());
    }
    return geo::heading(points.front(), points.back());
}

} // namespace xosm::procedural::geo_extra
