#pragma once

#include <iostream>
#include <sstream>
#include <string>

namespace osm2xodr::testing {

struct TestContext {
    int checks = 0;
    int failures = 0;
    std::string current_test;

    void check(const bool cond, const std::string& msg) {
        ++checks;
        if (!cond) {
            ++failures;
            std::cerr << "FAIL [" << current_test << "] " << msg << "\n";
        }
    }

    template <typename Fn>
    void run(const std::string& name, Fn&& fn) {
        current_test = name;
        fn(*this);
    }
};

} // namespace osm2xodr::testing

#define CHECK(ctx, cond) (ctx).check((cond), #cond)

#define CHECK_EQ(ctx, a, b)                                                        \
    do {                                                                           \
        const auto _a = (a);                                                       \
        const auto _b = (b);                                                       \
        if (!(_a == _b)) {                                                         \
            std::ostringstream _ss;                                                \
            _ss << #a " == " #b " (got " << _a << ", expected " << _b << ")";      \
            (ctx).check(false, _ss.str());                                         \
        } else {                                                                   \
            (ctx).check(true, #a " == " #b);                                       \
        }                                                                          \
    } while (0)
