#pragma once

#include <string>

inline std::string fixture(const std::string& name) {
    return std::string(PROCEDURAL_TEST_FIXTURES_DIR) + "/" + name;
}
