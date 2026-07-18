cmake_minimum_required(VERSION 3.20)
project(spite_d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dual-mode build.
#
# ROS mode (Linux): built by colcon; ament_cmake is present. Builds message
#   interfaces, core libraries, and node executables.
# Core mode (macOS dev): plain CMake; ament is absent. Builds only the
#   ROS-free core libraries, offline tools, and unit tests so the pipeline
#   logic can be developed and tested without a ROS installation.
#     cmake -B build -DSPITE_D_OPEN_SPITE_DIR=/path/to/open-spite \
#           -DCMAKE_TOOLCHAIN_FILE=/path/to/open-spite/build/conan_toolchain.cmake
# ---------------------------------------------------------------------------
find_package(ament_cmake QUIET)

# open-spite: consumed as a subdirectory. Its CGAL dependency must be
# resolvable (conan toolchain on macOS, system CGAL on the Linux box).
set(SPITE_D_OPEN_SPITE_DIR "" CACHE PATH "Path to the open-spite source tree")
if(SPITE_D_OPEN_SPITE_DIR)
  add_subdirectory(${SPITE_D_OPEN_SPITE_DIR} open_spite_build EXCLUDE_FROM_ALL)
  set(SPITE_D_HAVE_OPEN_SPITE TRUE)
else()
  message(STATUS "spite_d: SPITE_D_OPEN_SPITE_DIR not set; spite/planner cores skipped")
  set(SPITE_D_HAVE_OPEN_SPITE FALSE)
endif()

# ---------------------------------------------------------------------------
# Core libraries (ROS-free)
# ---------------------------------------------------------------------------
add_library(spite_d_core INTERFACE)
target_include_directories(spite_d_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(spite_d_trajectory
  src/trajectory/constant_velocity_predictor.cpp
)
target_link_libraries(spite_d_trajectory PUBLIC spite_d_core)

add_library(spite_d_planner
  src/planner/replanner.cpp
  src/planner/roadmap_io.cpp
)
target_link_libraries(spite_d_planner PUBLIC spite_d_core)

if(SPITE_D_HAVE_OPEN_SPITE)
  add_library(spite_d_validity
    src/spite/validity_server.cpp
  )
  target_link_libraries(spite_d_validity PUBLIC spite_d_core open_spite)
endif()

# Perception core (map_manager port) is added once the port lands:
# add_library(spite_d_perception src/perception/...)

# ---------------------------------------------------------------------------
# Offline tools
# ---------------------------------------------------------------------------
# build_roadmap needs OMPL (brew on macOS, ROS-provided on Linux). The
# roadmap graph is extracted via ompl::base::PlannerData, so any OMPL
# build works regardless of its Boost version.
find_package(ompl CONFIG QUIET)
if(SPITE_D_HAVE_OPEN_SPITE AND ompl_FOUND)
  add_executable(build_roadmap tools/build_roadmap.cpp)
  target_link_libraries(build_roadmap
    PRIVATE spite_d_planner spite_d_validity ompl::ompl)
else()
  message(STATUS "spite_d: build_roadmap tool skipped (needs open-spite + OMPL)")
endif()

# ---------------------------------------------------------------------------
# Tests (core mode and ROS mode)
# ---------------------------------------------------------------------------
enable_testing()
add_subdirectory(tests)

# ---------------------------------------------------------------------------
# ROS mode only: interfaces, nodes, install rules
# ---------------------------------------------------------------------------
if(ament_cmake_FOUND)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ObstacleState.msg"
    "msg/ObstacleArray.msg"
    "msg/PredictedTrajectory.msg"
    "msg/PredictedTrajectoryArray.msg"
    DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
  )

  # Node shims are added as they land:
  # add_executable(perception_node src/nodes/perception_node.cpp) ...

  install(DIRECTORY launch gazebo rviz
          DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY include/ DESTINATION include)

  ament_export_include_directories(include)
  ament_package()
endif()
