# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "arx_msgs: 4 messages, 0 services")

set(MSG_I_FLAGS "-Iarx_msgs:/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(arx_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/ChassisCtrl.msg" NAME_WE)
add_custom_target(_arx_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arx_msgs" "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/ChassisCtrl.msg" ""
)

get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointControl.msg" NAME_WE)
add_custom_target(_arx_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arx_msgs" "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointControl.msg" ""
)

get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointInformation.msg" NAME_WE)
add_custom_target(_arx_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arx_msgs" "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointInformation.msg" ""
)

get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/PosCmd.msg" NAME_WE)
add_custom_target(_arx_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arx_msgs" "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/PosCmd.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/ChassisCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arx_msgs
)
_generate_msg_cpp(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arx_msgs
)
_generate_msg_cpp(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointInformation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arx_msgs
)
_generate_msg_cpp(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/PosCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arx_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(arx_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arx_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(arx_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(arx_msgs_generate_messages arx_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/ChassisCtrl.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_cpp _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointControl.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_cpp _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointInformation.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_cpp _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/PosCmd.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_cpp _arx_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arx_msgs_gencpp)
add_dependencies(arx_msgs_gencpp arx_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arx_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/ChassisCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arx_msgs
)
_generate_msg_eus(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arx_msgs
)
_generate_msg_eus(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointInformation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arx_msgs
)
_generate_msg_eus(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/PosCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arx_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(arx_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arx_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(arx_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(arx_msgs_generate_messages arx_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/ChassisCtrl.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_eus _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointControl.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_eus _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointInformation.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_eus _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/PosCmd.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_eus _arx_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arx_msgs_geneus)
add_dependencies(arx_msgs_geneus arx_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arx_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/ChassisCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arx_msgs
)
_generate_msg_lisp(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arx_msgs
)
_generate_msg_lisp(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointInformation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arx_msgs
)
_generate_msg_lisp(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/PosCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arx_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(arx_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arx_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(arx_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(arx_msgs_generate_messages arx_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/ChassisCtrl.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_lisp _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointControl.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_lisp _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointInformation.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_lisp _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/PosCmd.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_lisp _arx_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arx_msgs_genlisp)
add_dependencies(arx_msgs_genlisp arx_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arx_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/ChassisCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arx_msgs
)
_generate_msg_nodejs(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arx_msgs
)
_generate_msg_nodejs(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointInformation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arx_msgs
)
_generate_msg_nodejs(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/PosCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arx_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(arx_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arx_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(arx_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(arx_msgs_generate_messages arx_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/ChassisCtrl.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_nodejs _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointControl.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_nodejs _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointInformation.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_nodejs _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/PosCmd.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_nodejs _arx_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arx_msgs_gennodejs)
add_dependencies(arx_msgs_gennodejs arx_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arx_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/ChassisCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_msgs
)
_generate_msg_py(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_msgs
)
_generate_msg_py(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointInformation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_msgs
)
_generate_msg_py(arx_msgs
  "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/PosCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(arx_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(arx_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(arx_msgs_generate_messages arx_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/ChassisCtrl.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_py _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointControl.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_py _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/JointInformation.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_py _arx_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/arm_pkg/src/arx/arx_msgs/msg/PosCmd.msg" NAME_WE)
add_dependencies(arx_msgs_generate_messages_py _arx_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arx_msgs_genpy)
add_dependencies(arx_msgs_genpy arx_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arx_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arx_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arx_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(arx_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arx_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arx_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(arx_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arx_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arx_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(arx_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arx_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arx_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(arx_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arx_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(arx_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
