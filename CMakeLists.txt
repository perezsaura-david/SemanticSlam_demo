cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_slam)
project(${PROJECT_NAME} VERSION 1.0.0)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
# list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_BUILD_TYPE Debug)


# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  pluginlib
  as2_core
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  fmt
  backward_ros
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

# G2O
find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
#EIGEN
include_directories("/usr/include/eigen3")
#FMT                      
set(FMT_LIBRARIES fmt::fmt)

# Plugins
# set(PLUGIN_LIST
#   raw_odometry
#   ground_truth
#   mocap_pose
# )

# Create a list of all the plugin files
# foreach(PLUGINS ${PLUGIN_LIST})
#   list(APPEND PLUGINS_CPP_FILES
#     plugins/${PLUGINS}/src/${PLUGINS}.cpp
#   )

#   include_directories(
#     plugins/${PLUGINS}/include
#     plugins/${PLUGINS}/include/${PLUGINS}
#   )
# endforeach()

# Library
# add_library(${PROJECT_NAME} SHARED ${PLUGINS_CPP_FILES})

# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
# pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#   TARGETS ${PROJECT_NAME}
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# ament_export_include_directories(
#   include
# )
# ament_export_libraries(
#   ${PROJECT_NAME}
# )
# ament_export_targets(
#   export_${PROJECT_NAME}
# )

set(SOURCE_CPP_FILES
  src/semantic_slam_node.cpp
  src/semantic_slam.cpp
  src/optimizer_g2o.cpp
  src/graph_g2o.cpp
  src/conversions.cpp
  src/debug_utils.cpp)

# Executable
add_executable(${PROJECT_NAME}_node ${SOURCE_CPP_FILES})
# add_executable(${PROJECT_NAME}_node src/semantic_slam_node.cpp src/semantic_slam.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_node
                      ${G2O_TYPES_DATA}
                      ${G2O_CORE_LIBRARY}
                      ${G2O_STUFF_LIBRARY}
                      ${G2O_SOLVER_PCG}
                      ${G2O_SOLVER_CSPARSE}   # be aware of that CSPARSE is released under LGPL
                      ${G2O_SOLVER_CHOLMOD}   # be aware of that cholmod is released under GPL
                      ${G2O_TYPES_SLAM3D}
                      ${G2O_TYPES_SLAM3D_ADDONS}
                      ${FMT_LIBRARIES}
                    )

add_executable(g2o_3D_slam src/g2o_3D_slam.cpp)
ament_target_dependencies(g2o_3D_slam ${PROJECT_DEPENDENCIES})
# target_link_libraries(g2o_3D_slam ${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(g2o_3D_slam
                      ${G2O_TYPES_DATA}
                      ${G2O_CORE_LIBRARY}
                      ${G2O_STUFF_LIBRARY}
                      ${G2O_SOLVER_PCG}
                      ${G2O_SOLVER_CSPARSE}   # be aware of that CSPARSE is released under LGPL
                      ${G2O_SOLVER_CHOLMOD}   # be aware of that cholmod is released under GPL
                      ${G2O_TYPES_SLAM3D}
                      ${G2O_TYPES_SLAM3D_ADDONS}
                      ${FMT_LIBRARIES}
                      )

install(TARGETS 
  ${PROJECT_NAME}_node
  g2o_3D_slam
  DESTINATION lib/${PROJECT_NAME})

# For each plugin, install config files
# foreach(PLUGINS ${PLUGIN_LIST})
#   if(EXISTS plugins/${PLUGINS}/config)
#     install(
#       DIRECTORY plugins/${PLUGINS}/config
#       DESTINATION share/${PROJECT_NAME}/plugins/${PLUGINS}
#     )
#   endif()
# endforeach()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_cppcheck(src/ include/ tests/ plugins/)
  ament_clang_format(src/ include/ tests/ plugins/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

  # Create a list of all the source files to be tested
  # from /plugins/plugin_name/tests/plugin_test.cpp if file exists
  # foreach(PLUGINS ${PLUGIN_LIST})
  #   if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/plugins/${PLUGINS}/tests/${PLUGINS}_test.cpp)
  #     list(APPEND TEST_SOURCE_CPP_FILES
  #       plugins/${PLUGINS}/tests/${PLUGINS}_test.cpp
  #     )
  #   endif()
  # endforeach()

  if(TEST_SOURCE_CPP_FILES)
    ament_add_gtest(${PROJECT_NAME}_test ${TEST_SOURCE_CPP_FILES})
    ament_target_dependencies(${PROJECT_NAME}_test ${PROJECT_DEPENDENCIES})
  endif()
endif()

ament_package()
