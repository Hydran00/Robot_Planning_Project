cmake_minimum_required(VERSION 3.12)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
  add_compile_options(-Wall -Wextra -Wpedantic -lboost_timer)
endif()
set(CMAKE_BUILD_TYPE "Release")
add_definitions(-w)
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")
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
find_package(ament_index_cpp)
find_package(Boost 1.74)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_executable(planner 
#   src/path_planner.cpp
# )
# ament_target_dependencies(planner 
#   rclcpp 
#   std_msgs
#   nav_msgs
#   nav2_msgs
#   geometry_msgs
#   tf2
#   tf2_ros
#   rclcpp_components
#   rclcpp_action
# )

add_executable(voronoi_boost
  src/voronoi_boost.cpp
)
ament_target_dependencies(voronoi_boost
  rclcpp
  ament_index_cpp
)
target_link_libraries(voronoi_boost Boost::headers)

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
  # planner
  # rec
  voronoi_boost
  DESTINATION lib/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/plot.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
