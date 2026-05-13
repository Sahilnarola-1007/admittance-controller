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

# Kortex API - adjust this path to your installation
# Typically installed via: sudo dpkg -i kortex_api_2.6.0-*.deb
# Or from source in your workspace
set(KORTEX_DIR "$ENV{HOME}/kortex_api" CACHE PATH "Path to Kortex API installation")

# Build the admittance controller node
add_executable(admittance_node
  src/admittance_node.cpp
)

target_include_directories(admittance_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${KORTEX_DIR}/include
  ${KORTEX_DIR}/include/client
  ${KORTEX_DIR}/include/common
  ${KORTEX_DIR}/include/messages
  ${KORTEX_DIR}/include/client_stubs
)

target_link_directories(admittance_node PUBLIC
  ${KORTEX_DIR}/lib
)

target_link_libraries(admittance_node
  KortexApiCpp
  pthread
)

ament_target_dependencies(admittance_node
  rclcpp
  geometry_msgs
  std_srvs
  rcl_interfaces
)

target_compile_features(admittance_node PUBLIC cxx_std_17)

# Install
install(TARGETS admittance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
