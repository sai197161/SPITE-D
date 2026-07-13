cmake_minimum_required(VERSION 3.16)
project(spite_d)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch gazebo rviz
        DESTINATION share/${PROJECT_NAME})

ament_package()