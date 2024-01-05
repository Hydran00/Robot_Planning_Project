cmake_minimum_required(VERSION 3.12)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
  add_compile_options(-Wall -Wextra -Wpedantic -lboost_timer)
endif()
set(CMAKE_BUILD_TYPE "Release")


add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")

##############################################################################

# Find ROS2 dependencies + Libraries

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp)
find_package(Boost 1.74)

#############################################################################

# Variables

set(VORONOI_FOLDER src/voronoi)
set(RRTSTAR_FOLDER src/rrt_star)

#############################################################################

# Voronoi Boost

add_executable(voronoi_boost
  ${VORONOI_FOLDER}/voronoi_boost.cpp
)

target_include_directories(voronoi_boost PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(voronoi_boost
  rclcpp
  ament_index_cpp
)
target_link_libraries(voronoi_boost Boost::headers)

add_executable(wkt_generator
  ${VORONOI_FOLDER}/wkt_generator.cpp
)
ament_target_dependencies(wkt_generator
  rclcpp
  obstacles_msgs
  geometry_msgs
  ament_index_cpp
)
target_include_directories(wkt_generator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

#############################################################################

# RRT
add_executable(rrt_star 
  ${RRTSTAR_FOLDER}/main.cpp
  ${RRTSTAR_FOLDER}/rrt.cpp 
  ${RRTSTAR_FOLDER}/rrt_plan.cpp 
  ${RRTSTAR_FOLDER}/rrt_star_plan.cpp
  ${RRTSTAR_FOLDER}/kdtree.cpp
  ${RRTSTAR_FOLDER}/map_info.cpp 
)

ament_target_dependencies(rrt_star
  rclcpp
  visualization_msgs
  geometry_msgs
  obstacles_msgs
)
target_link_libraries(rrt_star Boost::headers)
target_include_directories(rrt_star PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

#############################################################################


# Dubins

add_executable(dubins
  src/dubins/dubins.cpp
)

ament_target_dependencies(dubins
  rclcpp
  geometry_msgs
)

target_include_directories(dubins PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
#############################################################################


# Install folders

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY 
    scripts
    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY 
  data
  rviz
  DESTINATION share/${PROJECT_NAME})  

#############################################################################

# Install C++ executables

install(TARGETS
  voronoi_boost
  wkt_generator
  rrt_star
  dubins
  DESTINATION lib/${PROJECT_NAME})

#############################################################################

# Install Python executables

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/plot.py
  scripts/map_publisher.py
  scripts/visualise_dubins.py
  DESTINATION lib/${PROJECT_NAME}
)

#############################################################################

ament_package()
