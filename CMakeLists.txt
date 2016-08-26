cmake_minimum_required(VERSION 2.8.3)
project(imu_compass)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs std_msgs tf)

catkin_package()

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")
endif()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_compass src/imu_compass.cpp)
target_link_libraries(imu_compass ${catkin_LIBRARIES})

install(
  TARGETS imu_compass
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/compute_calibration
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
