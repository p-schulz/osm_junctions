add_executable(procedural_tests
    main.cpp
    test_control_lines.cpp
    test_control_points.cpp
    test_lanes.cpp
    test_graph.cpp
    test_integration.cpp
)
target_link_libraries(procedural_tests PRIVATE osm2xodr_procedural)
target_include_directories(procedural_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(procedural_tests PRIVATE
    PROCEDURAL_TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")

add_test(NAME procedural_tests COMMAND procedural_tests)
