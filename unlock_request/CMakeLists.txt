cmake_minimum_required(VERSION 3.8)
project(unlock_request)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rapidjson REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(protocol REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cpp_httplib REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(motion_utils REQUIRED)
find_package(low_power_consumption REQUIRED)
find_package(bes_transmit REQUIRED)
find_package(black_box REQUIRED)

include_directories(include)
include_directories(${OPENSSL_INCLUDE_DIRS})

install(
  DIRECTORY include/
  DESTINATION include/
)

add_executable(unlock_request src/main.cpp)
target_link_libraries(${PROJECT_NAME} OpenSSL::Crypto OpenSSL::SSL)
target_link_libraries(${PROJECT_NAME} pthread)
ament_target_dependencies(${PROJECT_NAME}
  rapidjson
  cyberdog_common
  rclcpp
  protocol
  std_srvs
  cpp_httplib
  motion_utils
  low_power_consumption
  bes_transmit
  black_box
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
