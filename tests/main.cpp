#include "test_framework.hpp"

#include <iostream>

using xosm::testing::TestContext;

void run_control_line_tests(TestContext&);
void run_control_point_tests(TestContext&);
void run_lane_tests(TestContext&);
void run_graph_tests(TestContext&);
void run_intersection_tests(TestContext&);
void run_integration_tests(TestContext&);

int main() {
    TestContext ctx;
    run_control_line_tests(ctx);
    run_control_point_tests(ctx);
    run_lane_tests(ctx);
    run_graph_tests(ctx);
    run_intersection_tests(ctx);
    run_integration_tests(ctx);

    std::cerr << "\n" << ctx.checks << " check(s), " << ctx.failures << " failure(s).\n";
    return ctx.failures == 0 ? 0 : 1;
}
