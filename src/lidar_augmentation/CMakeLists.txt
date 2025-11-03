cmake_minimum_required(VERSION 3.0.2)
project(lidar_augmentation)

## Compile as C++17 for modern features and performance
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Enable optimizations for performance (critical for 25-30Hz goal)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -Wall -Wextra")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  message_filters
  pcl_conversions
  pcl_ros
  roslib
)

## Find system dependencies - CRITICAL for our C++ implementation
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

## PCL causes some warnings we can safely ignore
add_definitions(${PCL_DEFINITIONS})

## Python setup for existing Python tools (keeping compatibility)
#catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES 
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS 
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_eigen
    message_filters
    pcl_conversions
    pcl_ros
    roslib
  DEPENDS 
    PCL
    Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Create core library with all our C++ implementations
add_library(${PROJECT_NAME}_core
  src/point_cloud_processor.cpp
  src/augmentation_methods.cpp
  src/imu_synchronizer.cpp
  src/lidar_augmenter_node.cpp
)

## Link core library with all dependencies
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
)

## Ensure proper linking order for PCL
target_link_directories(${PROJECT_NAME}_core PRIVATE ${PCL_LIBRARY_DIRS})

## Main executable - LiDAR Augmenter Node (C++)
add_executable(lidar_augmenter_node
  src/lidar_augmenter_node_main.cpp
)

## Link executable with core library
target_link_libraries(lidar_augmenter_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
)

## Set target properties for optimization
set_target_properties(lidar_augmenter_node PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Additional executables for utilities (if needed later)
# add_executable(performance_tester
#   src/cpp/tools/performance_tester.cpp
# )
# target_link_libraries(performance_tester
#   ${PROJECT_NAME}_core
#   ${catkin_LIBRARIES}
# )

## Compiler-specific optimizations
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME}_core PRIVATE
    -Wall -Wextra -Wpedantic
    -O3 -funroll-loops -ffast-math
    -DEIGEN_NO_DEBUG -DEIGEN_DONT_PARALLELIZE
  )
  target_compile_options(lidar_augmenter_node PRIVATE
    -Wall -Wextra -Wpedantic
    -O3 -funroll-loops -ffast-math
  )
endif()

## Template instantiation optimization
target_compile_definitions(${PROJECT_NAME}_core PRIVATE
  PCL_NO_PRECOMPILE
)

#############
## Install ##
#############

## Install C++ executables
install(TARGETS lidar_augmenter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install C++ library
install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install C++ headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## Install Python modules (keeping existing Python tools)
#catkin_install_python(PROGRAMS
#  scripts/visualize_augmentation.py
#  scripts/setup_rviz_config.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

## Install rviz configs
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

#############
## Testing ##
#############

## Unit tests for C++ components
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  # Test point cloud processor
  catkin_add_gtest(test_point_cloud_processor
    test/cpp/test_point_cloud_processor.cpp
  )
  if(TARGET test_point_cloud_processor)
    target_link_libraries(test_point_cloud_processor
      ${PROJECT_NAME}_core
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()
  
  # Test augmentation methods
  catkin_add_gtest(test_augmentation_methods
    test/cpp/test_augmentation_methods.cpp
  )
  if(TARGET test_augmentation_methods)
    target_link_libraries(test_augmentation_methods
      ${PROJECT_NAME}_core
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()
  
  # ✅ ADD THIS: Integration test with ROS
  add_rostest_gtest(test_integration
    test/launch/test_integration.launch
    test/cpp/test_integration.cpp
  )
  if(TARGET test_integration)
    target_link_libraries(test_integration
      ${PROJECT_NAME}_core
      ${catkin_LIBRARIES}
    )
  endif()
  
endif()

#################
## Development ##
#################

## Generate compile commands for development tools (clangd, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Debug information
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "=== LiDAR Augmentation Build Configuration ===")
  message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
  message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
  message(STATUS "PCL Version: ${PCL_VERSION}")
  message(STATUS "Eigen Version: ${EIGEN3_VERSION_STRING}")
  message(STATUS "Boost Version: ${Boost_VERSION}")
  message(STATUS "YAML-CPP Found: ${yaml-cpp_FOUND}")
  message(STATUS "===============================================")
endif()

## Performance profiling support (optional)
option(ENABLE_PROFILING "Enable performance profiling" OFF)
if(ENABLE_PROFILING)
  target_compile_definitions(${PROJECT_NAME}_core PRIVATE ENABLE_PROFILING)
  target_link_libraries(${PROJECT_NAME}_core profiler)
endif()

## Memory debugging support (optional)
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
if(ENABLE_ASAN AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(${PROJECT_NAME}_core PRIVATE -fsanitize=address)
  target_link_options(${PROJECT_NAME}_core PRIVATE -fsanitize=address)
endif()

##################
## Documentation ##
##################

## Doxygen documentation (optional)
#find_package(Doxygen)
#if(DOXYGEN_FOUND)
#  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
#    ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
#  add_custom_target(doc
#    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
#    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#    COMMENT "Generating API documentation with Doxygen" VERBATIM
#  )
#endif()

#####################
## Troubleshooting ##
#####################

## Common build issue fixes
if(NOT PCL_FOUND)
  message(FATAL_ERROR "PCL not found! Install with: sudo apt-get install libpcl-dev")
endif()

if(NOT EIGEN3_FOUND)
  message(FATAL_ERROR "Eigen3 not found! Install with: sudo apt-get install libeigen3-dev")
endif()

if(NOT yaml-cpp_FOUND)
  message(FATAL_ERROR "yaml-cpp not found! Install with: sudo apt-get install libyaml-cpp-dev")
endif()

## Ensure C++17 support
if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "7.0")
  message(WARNING "GCC version ${CMAKE_CXX_COMPILER_VERSION} may not fully support C++17. Recommend GCC 7+")
endif()

## Link order fix for some systems
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  target_link_libraries(lidar_augmenter_node -Wl,--no-as-needed)
endif()

message(STATUS "LiDAR Augmentation package configured successfully!")
message(STATUS "Target executable: lidar_augmenter_node")
message(STATUS "Expected performance: 25-30Hz point cloud processing")
