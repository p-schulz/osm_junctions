add_executable(xosm_tests
    main.cpp
    test_control_lines.cpp
    test_control_points.cpp
    test_lanes.cpp
    test_graph.cpp
    test_integration.cpp
)
target_link_libraries(xosm_tests PRIVATE xosm_procedural)
target_include_directories(xosm_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(xosm_tests PRIVATE
    PROCEDURAL_TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")

add_test(NAME xosm_tests COMMAND xosm_tests)
