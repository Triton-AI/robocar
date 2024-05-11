cmake_minimum_required(VERSION 3.5)
project(twist_to_ackermann)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

### ------------------
### find dependencies
### ------------------

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# add node entries, creating executables
add_executable(twist_to_ackermann_node src/twist_to_ackermann_node.cpp)
ament_target_dependencies(twist_to_ackermann_node rclcpp std_msgs geometry_msgs ackermann_msgs)

install(TARGETS
  twist_to_ackermann_node
  DESTINATION lib/${PROJECT_NAME}  
)

# target_include_directories(my_target
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)

### ---------------------------
### install additional folders
### ---------------------------

# install(DIRECTORY
#   <desired folder>
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
	launch
  param
)
