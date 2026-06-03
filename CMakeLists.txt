cmake_minimum_required(VERSION 3.8)
project(admittance_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(kinova_wrapper REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wipe_msgs REQUIRED)

# Build the admittance controller node
add_executable(admittance_node
  src/admittance_node.cpp
)

target_include_directories(admittance_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  
)

ament_target_dependencies(admittance_node
  rclcpp
  geometry_msgs
  std_srvs
  rcl_interfaces
  kinova_wrapper
  Eigen3
  wipe_msgs
)

target_compile_features(admittance_node PUBLIC cxx_std_17)

# Install
install(TARGETS admittance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_force_controller_pi test/test_force_controller_pi.cpp)
  target_include_directories(test_force_controller_pi PRIVATE include)
  target_compile_features(test_force_controller_pi PUBLIC cxx_std_17)
endif()

ament_package()
