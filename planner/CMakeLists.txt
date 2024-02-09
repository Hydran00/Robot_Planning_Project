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
set(RRT_FOLDER src/rrt)

#############################################################################

# Voronoi Boost

add_executable(voronoi_planner
  ${VORONOI_FOLDER}/main_voronoi.cpp
  ${VORONOI_FOLDER}/utils/dijkstra.cpp
  ${VORONOI_FOLDER}/utils/voronoi_builder.cpp
  ${VORONOI_FOLDER}/utils/random_combinations.cpp
  ${VORONOI_FOLDER}/planners/voronoi_plan.cpp
  src/map_info_node.cpp 
  src/dubins/dubins.cpp
  # these are needed to compile map_info_node
  ${RRT_FOLDER}/utils/kdtree.cpp
  ${RRT_FOLDER}/utils/rrt.cpp
  

)

ament_target_dependencies(voronoi_planner
  rclcpp
  visualization_msgs
  geometry_msgs
  obstacles_msgs
  tf2
  tf2_ros
  nav_msgs
)
target_link_libraries(voronoi_planner Boost::headers)
target_include_directories(voronoi_planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

#############################################################################

# RRT*
add_executable(rrt_star 
  ${RRT_FOLDER}/main_rrt_star.cpp
  ${RRT_FOLDER}/planners/rrt_plan.cpp 
  ${RRT_FOLDER}/utils/kdtree.cpp
  ${RRT_FOLDER}/planners/rrt_star_plan.cpp
  ${RRT_FOLDER}/utils/rrt.cpp 
  src/map_info_node.cpp 
  src/dubins/dubins.cpp
  # ${RRT_FOLDER}/utils/rrt_dubins.cpp 
  # ${RRT_FOLDER}/planners/rrt_star_dubins_plan.cpp
  # src/dubins/dubins.cpp 
)

ament_target_dependencies(rrt_star
  rclcpp
  visualization_msgs
  geometry_msgs
  obstacles_msgs
  tf2
  tf2_ros
  nav_msgs
)
target_link_libraries(rrt_star Boost::headers)
target_include_directories(rrt_star PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

#############################################################################
# RRT* Dubins

# RRT
add_executable(rrt_star_dubins
  ${RRT_FOLDER}/main_rrt_star_dubins.cpp
  src/map_info_node.cpp 
  ${RRT_FOLDER}/utils/kdtree.cpp
  ${RRT_FOLDER}/utils/rrt_dubins.cpp 
  ${RRT_FOLDER}/planners/rrt_star_dubins_plan.cpp
  ${RRT_FOLDER}/utils/rrt.cpp
  ${RRT_FOLDER}/planners/rrt_plan.cpp 
  src/dubins/dubins.cpp
  )

ament_target_dependencies(rrt_star_dubins
  rclcpp
  visualization_msgs
  geometry_msgs
  obstacles_msgs
  tf2
  tf2_ros
  nav_msgs
)
target_link_libraries(rrt_star_dubins Boost::headers)
target_include_directories(rrt_star_dubins PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

#############################################################################

# RRT* Dubins (Multithread)

# RRT
add_executable(multithread_rrt_star_dubins
  ${RRT_FOLDER}/main_multithreaded.cpp
  # ${RRT_FOLDER}/threaded_planner.cpp
  src/map_info_node.cpp 
  ${RRT_FOLDER}/utils/kdtree.cpp
  ${RRT_FOLDER}/utils/rrt.cpp
  ${RRT_FOLDER}/planners/rrt_plan.cpp 
  ${RRT_FOLDER}/planners/rrt_star_plan.cpp 
  ${RRT_FOLDER}/utils/rrt_dubins.cpp 
  ${RRT_FOLDER}/planners/rrt_star_dubins_plan.cpp
  src/dubins/dubins.cpp
  )

ament_target_dependencies(multithread_rrt_star_dubins
  rclcpp
  visualization_msgs
  geometry_msgs
  obstacles_msgs
  tf2
  tf2_ros
  nav_msgs
)
target_link_libraries(multithread_rrt_star_dubins Boost::headers)
target_include_directories(multithread_rrt_star_dubins PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

#############################################################################

# Path Planner

add_executable(nav2_client
src/nav2_client.cpp
)

ament_target_dependencies(nav2_client
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  nav2_msgs
)
# target_include_directories(path_planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

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
  voronoi_planner
  # wkt_generator
  rrt_star
  rrt_star_dubins
  multithread_rrt_star_dubins
  # dubins
  nav2_client
  DESTINATION lib/${PROJECT_NAME})

#############################################################################

# Install Python executables

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/plot.py
  scripts/map_publisher.py
  scripts/visualise_dubins.py
  scripts/visualise_voronoi.py
  DESTINATION lib/${PROJECT_NAME}
)

#############################################################################

ament_package()
