# Install script for directory: /home/user/aerostack2_ws/src/aerostack2/as2_slam

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/as2_slam_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/as2_slam_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/as2_slam_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/as2_slam" TYPE EXECUTABLE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/as2_slam_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/as2_slam_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/as2_slam_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/as2_slam_node"
         OLD_RPATH "/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/user/aerostack2_ws/install/g2o/lib:/home/user/aerostack2_ws/install/as2_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/as2_slam_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/g2o_3D_slam" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/g2o_3D_slam")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/g2o_3D_slam"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/as2_slam" TYPE EXECUTABLE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/g2o_3D_slam")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/g2o_3D_slam" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/g2o_3D_slam")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/g2o_3D_slam"
         OLD_RPATH "/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/user/aerostack2_ws/install/g2o/lib:/home/user/aerostack2_ws/install/as2_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/as2_slam/g2o_3D_slam")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/as2_slam" TYPE DIRECTORY FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/as2_slam")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/as2_slam")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/as2_slam/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/as2_slam/environment" TYPE FILE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/as2_slam/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/as2_slam/environment" TYPE FILE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/as2_slam" TYPE FILE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/as2_slam" TYPE FILE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/as2_slam" TYPE FILE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/as2_slam" TYPE FILE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/as2_slam" TYPE FILE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/ament_cmake_index/share/ament_index/resource_index/packages/as2_slam")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/as2_slam/cmake" TYPE FILE FILES
    "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/ament_cmake_core/as2_slamConfig.cmake"
    "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/ament_cmake_core/as2_slamConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/as2_slam" TYPE FILE FILES "/home/user/aerostack2_ws/src/aerostack2/as2_slam/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/user/aerostack2_ws/src/aerostack2/as2_slam/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
