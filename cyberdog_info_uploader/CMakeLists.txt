cmake_minimum_required(VERSION 3.8)
project(cyberdog_info_uploader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(protocol REQUIRED)
find_package(cyberdog_common REQUIRED)

include_directories(include)
add_library(cyberdog_info_uploader SHARED
  src/cyberdog_info_uploader.cpp
  src/cyberdog_log_uploading.cpp
)
target_include_directories(cyberdog_info_uploader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cyberdog_info_uploader PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(cyberdog_info_uploader
  rclcpp
  protocol
  cyberdog_common
)
ament_export_targets(cyberdog_info_uploader HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs protocol cyberdog_common)
install(TARGETS cyberdog_info_uploader
  EXPORT cyberdog_info_uploader
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
