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


add_executable(voronoi_boost
  src/voronoi_boost.cpp
)


# Voronoi Boost
target_include_directories(voronoi_boost PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(voronoi_boost
rclcpp
ament_index_cpp
)
target_link_libraries(voronoi_boost Boost::headers)


# Planner
add_executable(planner
  src/planner.cpp
)
ament_target_dependencies(planner
  rclcpp
  ament_index_cpp
)
target_include_directories(planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Install 
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY 
    scripts
    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY 
  data
  DESTINATION share/${PROJECT_NAME})  
install(TARGETS
  planner
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
