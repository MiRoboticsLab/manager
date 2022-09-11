cmake_minimum_required(VERSION 3.8)
project(user_info_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-class-memaccess)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rapidjson REQUIRED)
find_package(cyberdog_common REQUIRED)

set(dependencies
  cyberdog_common
  rapidjson
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(include)

add_executable(node_test src/test_inteface_node.cpp)
add_executable(test1 test/test.cpp)

#target_compile_definitions(test_node PRIVATE BenchmarkPath="${CMAKE_INSTALL_PREFIX}/lib/cyberdog_common")
ament_target_dependencies(node_test rclcpp rapidjson cyberdog_common)
ament_target_dependencies(test1 rapidjson cyberdog_common)

target_link_libraries(node_test ${cyberdog_log_LIBRARIES})
target_link_libraries(test1 ${cyberdog_log_LIBRARIES})

install(TARGETS
  node_test
  test1
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()

