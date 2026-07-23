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

add_executable(test_span test_span.cpp)
target_link_libraries(test_span PRIVATE spite_d_dynamic_map)
add_test(NAME test_span COMMAND test_span)

if(TARGET spite_d_validity)
  add_executable(test_span_pipeline test_span_pipeline.cpp)
  target_link_libraries(test_span_pipeline
    PRIVATE spite_d_dynamic_map spite_d_validity spite_d_trajectory)
  add_test(NAME test_span_pipeline COMMAND test_span_pipeline)
endif()

if(TARGET spite_d_perception)
  add_executable(test_perception test_perception.cpp)
  target_link_libraries(test_perception PRIVATE spite_d_perception)
  add_test(NAME test_perception COMMAND test_perception)
endif()
