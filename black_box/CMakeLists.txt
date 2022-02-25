cmake_minimum_required(VERSION 3.8)
project(black_box)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(protocol REQUIRED)
find_package(sqlite3_vendor REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(params REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_library(black_box SHARED src/black_box.cpp)
target_link_libraries(black_box ${SQLite3_LIBRARIES})
ament_target_dependencies(black_box rclcpp protocol cyberdog_common params)

# target_include_directories(black_box
#                            PUBLIC
#                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#                            $<INSTALL_INTERFACE:include>)

add_executable(test_black_box src/test_black_box.cpp)
target_link_libraries(test_black_box black_box)
ament_target_dependencies(test_black_box rclcpp)
# target_include_directories(test_black_box
#                            PUBLIC
#                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#                            $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
        include/
        DESTINATION include)

install(TARGETS black_box test_black_box
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_dependencies(SQLite3)
ament_export_libraries(black_box)
ament_package()
