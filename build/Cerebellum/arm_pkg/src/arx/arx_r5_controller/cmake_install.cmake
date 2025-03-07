# Install script for directory: /home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_controller

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zjy/YanBot/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zjy/YanBot/build/Cerebellum/arm_pkg/src/arx/arx_r5_controller/catkin_generated/installspace/arx_r5_controller.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arx_r5_controller/cmake" TYPE FILE FILES
    "/home/zjy/YanBot/build/Cerebellum/arm_pkg/src/arx/arx_r5_controller/catkin_generated/installspace/arx_r5_controllerConfig.cmake"
    "/home/zjy/YanBot/build/Cerebellum/arm_pkg/src/arx/arx_r5_controller/catkin_generated/installspace/arx_r5_controllerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arx_r5_controller" TYPE FILE FILES "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_controller/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5Controller" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5Controller")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5Controller"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller" TYPE EXECUTABLE FILES "/home/zjy/YanBot/devel/lib/arx_r5_controller/R5Controller")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5Controller" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5Controller")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5Controller"
         OLD_RPATH "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_controller/lib:/opt/ros/noetic/lib:/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_controller/lib/arx_r5_src:/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_controller/lib/arx_hardware_interface:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5Controller")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5ControllerVr" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5ControllerVr")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5ControllerVr"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller" TYPE EXECUTABLE FILES "/home/zjy/YanBot/devel/lib/arx_r5_controller/R5ControllerVr")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5ControllerVr" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5ControllerVr")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5ControllerVr"
         OLD_RPATH "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_controller/lib:/opt/ros/noetic/lib:/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_controller/lib/arx_r5_src:/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_controller/lib/arx_hardware_interface:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/arx_r5_controller/R5ControllerVr")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE FILE FILES
    "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_controller/lib/arx_r5_src/libarx_r5_src.so"
    "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_controller/lib/arx_hardware_interface/libarx_hardware_interface.so"
    )
endif()

