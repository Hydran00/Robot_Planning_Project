cmake_minimum_required(VERSION 3.12)
project(dubins_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev)
endif()
set(CMAKE_BUILD_TYPE "Release")
set(CGAL_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/data)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(CGAL REQUIRED)
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)
find_package(ament_index_cpp)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(dubins_planner 
  src/path_planner.cpp
)
ament_target_dependencies(dubins_planner 
  rclcpp 
  std_msgs
  nav_msgs
  nav2_msgs
  geometry_msgs
  tf2
  tf2_ros
  rclcpp_components
  rclcpp_action
)
add_executable(rec src/rec.cpp)
  ament_target_dependencies(rec
  rclcpp
  ament_index_cpp
)
#link it with the required CGAL libraries

target_link_libraries(rec CGAL::CGAL CGAL::CGAL_Basic_viewer)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY 
    scripts
    DESTINATION share/${PROJECT_NAME})

# send data to share directory
install(DIRECTORY 
  data
  DESTINATION share/${PROJECT_NAME})
  
install(TARGETS
  dubins_planner
  rec
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/plot.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
