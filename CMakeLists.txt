cmake_minimum_required(VERSION 3.5)
project(reachability_analysis)

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
  rclcpp
  ocs2_ros_interfaces
  ocs2_go2_mpc
  ocs2_robotic_tools
)


# Find ROS2 build system
find_package(ament_cmake REQUIRED)

# Need now, not sure why
find_package(Boost REQUIRED COMPONENTS 
              system 
              filesystem
              log 
              log_setup)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Dependencies
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

# Main library
add_library(${PROJECT_NAME}
        src/ReachabilityAnalyzer.cpp
)
ament_target_dependencies(${PROJECT_NAME} 
	${dependencies}
)
        
# reachability analysis node
add_executable(legged_robot_reachability_analysis
                src/LeggedRobotReachabilityAnalysisNode.cpp
)
ament_target_dependencies(legged_robot_reachability_analysis
	${dependencies}
)
target_link_libraries(legged_robot_reachability_analysis
        ${PROJECT_NAME}
        )

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS 
        legged_robot_reachability_analysis
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()