# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "grounding_sam_ros: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(grounding_sam_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/VitDetection.srv" NAME_WE)
add_custom_target(_grounding_sam_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "grounding_sam_ros" "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/VitDetection.srv" "std_msgs/MultiArrayDimension:sensor_msgs/Image:std_msgs/Header:std_msgs/MultiArrayLayout:std_msgs/Float32MultiArray"
)

get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/UpdatePrompt.srv" NAME_WE)
add_custom_target(_grounding_sam_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "grounding_sam_ros" "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/UpdatePrompt.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(grounding_sam_ros
  "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/VitDetection.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grounding_sam_ros
)
_generate_srv_cpp(grounding_sam_ros
  "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/UpdatePrompt.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grounding_sam_ros
)

### Generating Module File
_generate_module_cpp(grounding_sam_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grounding_sam_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(grounding_sam_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(grounding_sam_ros_generate_messages grounding_sam_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/VitDetection.srv" NAME_WE)
add_dependencies(grounding_sam_ros_generate_messages_cpp _grounding_sam_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/UpdatePrompt.srv" NAME_WE)
add_dependencies(grounding_sam_ros_generate_messages_cpp _grounding_sam_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grounding_sam_ros_gencpp)
add_dependencies(grounding_sam_ros_gencpp grounding_sam_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grounding_sam_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(grounding_sam_ros
  "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/VitDetection.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grounding_sam_ros
)
_generate_srv_eus(grounding_sam_ros
  "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/UpdatePrompt.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grounding_sam_ros
)

### Generating Module File
_generate_module_eus(grounding_sam_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grounding_sam_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(grounding_sam_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(grounding_sam_ros_generate_messages grounding_sam_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/VitDetection.srv" NAME_WE)
add_dependencies(grounding_sam_ros_generate_messages_eus _grounding_sam_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/UpdatePrompt.srv" NAME_WE)
add_dependencies(grounding_sam_ros_generate_messages_eus _grounding_sam_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grounding_sam_ros_geneus)
add_dependencies(grounding_sam_ros_geneus grounding_sam_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grounding_sam_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(grounding_sam_ros
  "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/VitDetection.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grounding_sam_ros
)
_generate_srv_lisp(grounding_sam_ros
  "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/UpdatePrompt.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grounding_sam_ros
)

### Generating Module File
_generate_module_lisp(grounding_sam_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grounding_sam_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(grounding_sam_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(grounding_sam_ros_generate_messages grounding_sam_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/VitDetection.srv" NAME_WE)
add_dependencies(grounding_sam_ros_generate_messages_lisp _grounding_sam_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/UpdatePrompt.srv" NAME_WE)
add_dependencies(grounding_sam_ros_generate_messages_lisp _grounding_sam_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grounding_sam_ros_genlisp)
add_dependencies(grounding_sam_ros_genlisp grounding_sam_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grounding_sam_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(grounding_sam_ros
  "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/VitDetection.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/grounding_sam_ros
)
_generate_srv_nodejs(grounding_sam_ros
  "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/UpdatePrompt.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/grounding_sam_ros
)

### Generating Module File
_generate_module_nodejs(grounding_sam_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/grounding_sam_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(grounding_sam_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(grounding_sam_ros_generate_messages grounding_sam_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/VitDetection.srv" NAME_WE)
add_dependencies(grounding_sam_ros_generate_messages_nodejs _grounding_sam_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/UpdatePrompt.srv" NAME_WE)
add_dependencies(grounding_sam_ros_generate_messages_nodejs _grounding_sam_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grounding_sam_ros_gennodejs)
add_dependencies(grounding_sam_ros_gennodejs grounding_sam_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grounding_sam_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(grounding_sam_ros
  "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/VitDetection.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grounding_sam_ros
)
_generate_srv_py(grounding_sam_ros
  "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/UpdatePrompt.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grounding_sam_ros
)

### Generating Module File
_generate_module_py(grounding_sam_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grounding_sam_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(grounding_sam_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(grounding_sam_ros_generate_messages grounding_sam_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/VitDetection.srv" NAME_WE)
add_dependencies(grounding_sam_ros_generate_messages_py _grounding_sam_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zjy/YanBot/src/Cerebellum/grounding_sam_ros_pkg/srv/UpdatePrompt.srv" NAME_WE)
add_dependencies(grounding_sam_ros_generate_messages_py _grounding_sam_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grounding_sam_ros_genpy)
add_dependencies(grounding_sam_ros_genpy grounding_sam_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grounding_sam_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grounding_sam_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grounding_sam_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(grounding_sam_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(grounding_sam_ros_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grounding_sam_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grounding_sam_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(grounding_sam_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(grounding_sam_ros_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grounding_sam_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grounding_sam_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(grounding_sam_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(grounding_sam_ros_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/grounding_sam_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/grounding_sam_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(grounding_sam_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(grounding_sam_ros_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grounding_sam_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grounding_sam_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grounding_sam_ros
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grounding_sam_ros")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grounding_sam_ros
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(grounding_sam_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(grounding_sam_ros_generate_messages_py sensor_msgs_generate_messages_py)
endif()
