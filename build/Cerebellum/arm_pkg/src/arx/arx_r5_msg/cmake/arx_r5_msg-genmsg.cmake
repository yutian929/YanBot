# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "arx_r5_msg: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iarx_r5_msg:/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(arx_r5_msg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotCmd.msg" NAME_WE)
add_custom_target(_arx_r5_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arx_r5_msg" "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotCmd.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotStatus.msg" NAME_WE)
add_custom_target(_arx_r5_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arx_r5_msg" "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotStatus.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(arx_r5_msg
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotCmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arx_r5_msg
)
_generate_msg_cpp(arx_r5_msg
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arx_r5_msg
)

### Generating Services

### Generating Module File
_generate_module_cpp(arx_r5_msg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arx_r5_msg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(arx_r5_msg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(arx_r5_msg_generate_messages arx_r5_msg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotCmd.msg" NAME_WE)
add_dependencies(arx_r5_msg_generate_messages_cpp _arx_r5_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotStatus.msg" NAME_WE)
add_dependencies(arx_r5_msg_generate_messages_cpp _arx_r5_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arx_r5_msg_gencpp)
add_dependencies(arx_r5_msg_gencpp arx_r5_msg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arx_r5_msg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(arx_r5_msg
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotCmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arx_r5_msg
)
_generate_msg_eus(arx_r5_msg
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arx_r5_msg
)

### Generating Services

### Generating Module File
_generate_module_eus(arx_r5_msg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arx_r5_msg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(arx_r5_msg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(arx_r5_msg_generate_messages arx_r5_msg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotCmd.msg" NAME_WE)
add_dependencies(arx_r5_msg_generate_messages_eus _arx_r5_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotStatus.msg" NAME_WE)
add_dependencies(arx_r5_msg_generate_messages_eus _arx_r5_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arx_r5_msg_geneus)
add_dependencies(arx_r5_msg_geneus arx_r5_msg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arx_r5_msg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(arx_r5_msg
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotCmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arx_r5_msg
)
_generate_msg_lisp(arx_r5_msg
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arx_r5_msg
)

### Generating Services

### Generating Module File
_generate_module_lisp(arx_r5_msg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arx_r5_msg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(arx_r5_msg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(arx_r5_msg_generate_messages arx_r5_msg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotCmd.msg" NAME_WE)
add_dependencies(arx_r5_msg_generate_messages_lisp _arx_r5_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotStatus.msg" NAME_WE)
add_dependencies(arx_r5_msg_generate_messages_lisp _arx_r5_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arx_r5_msg_genlisp)
add_dependencies(arx_r5_msg_genlisp arx_r5_msg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arx_r5_msg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(arx_r5_msg
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotCmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arx_r5_msg
)
_generate_msg_nodejs(arx_r5_msg
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arx_r5_msg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(arx_r5_msg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arx_r5_msg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(arx_r5_msg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(arx_r5_msg_generate_messages arx_r5_msg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotCmd.msg" NAME_WE)
add_dependencies(arx_r5_msg_generate_messages_nodejs _arx_r5_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotStatus.msg" NAME_WE)
add_dependencies(arx_r5_msg_generate_messages_nodejs _arx_r5_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arx_r5_msg_gennodejs)
add_dependencies(arx_r5_msg_gennodejs arx_r5_msg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arx_r5_msg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(arx_r5_msg
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotCmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_r5_msg
)
_generate_msg_py(arx_r5_msg
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_r5_msg
)

### Generating Services

### Generating Module File
_generate_module_py(arx_r5_msg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_r5_msg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(arx_r5_msg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(arx_r5_msg_generate_messages arx_r5_msg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotCmd.msg" NAME_WE)
add_dependencies(arx_r5_msg_generate_messages_py _arx_r5_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_r5_msg/msg/RobotStatus.msg" NAME_WE)
add_dependencies(arx_r5_msg_generate_messages_py _arx_r5_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arx_r5_msg_genpy)
add_dependencies(arx_r5_msg_genpy arx_r5_msg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arx_r5_msg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arx_r5_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arx_r5_msg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(arx_r5_msg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arx_r5_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arx_r5_msg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(arx_r5_msg_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arx_r5_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arx_r5_msg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(arx_r5_msg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arx_r5_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arx_r5_msg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(arx_r5_msg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_r5_msg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_r5_msg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_r5_msg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(arx_r5_msg_generate_messages_py std_msgs_generate_messages_py)
endif()
