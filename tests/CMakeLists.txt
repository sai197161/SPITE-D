# Plain-assert unit tests; no framework dependency so they run anywhere.
add_executable(test_predictor test_predictor.cpp)
target_link_libraries(test_predictor PRIVATE spite_d_trajectory)
add_test(NAME test_predictor COMMAND test_predictor)

add_executable(test_replanner test_replanner.cpp)
target_link_libraries(test_replanner PRIVATE spite_d_planner)
add_test(NAME test_replanner COMMAND test_replanner)

if(TARGET spite_d_validity)
  add_executable(test_validity_server test_validity_server.cpp)
  target_link_libraries(test_validity_server
    PRIVATE spite_d_validity spite_d_trajectory spite_d_planner)
  add_test(NAME test_validity_server COMMAND test_validity_server)

  add_executable(test_roadmap_roundtrip test_roadmap_roundtrip.cpp)
  target_link_libraries(test_roadmap_roundtrip
    PRIVATE spite_d_validity spite_d_trajectory spite_d_planner)
  add_test(NAME test_roadmap_roundtrip COMMAND test_roadmap_roundtrip)
endif()
