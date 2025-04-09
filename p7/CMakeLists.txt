cmake_minimum_required(VERSION 3.8)
project(p7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  vision_msgs
  tf2_ros
  tf2_geometry_msgs
  yolo_msgs
  rclcpp_lifecycle
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/Life_cycle_control/Detection.cpp
  src/Life_cycle_control/PIDController.cpp
  src/Life_cycle_control/Follow.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(Detecion src/Detection_main.cpp)
ament_target_dependencies(Detecion  ${dependencies})
target_link_libraries(Detecion  ${PROJECT_NAME})

add_executable(Busca src/seeker.cpp)
ament_target_dependencies(Busca  ${dependencies})
target_link_libraries(Busca  ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  Detecion
  Busca
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()