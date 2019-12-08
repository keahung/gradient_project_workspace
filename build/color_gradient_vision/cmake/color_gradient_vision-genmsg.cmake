# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "color_gradient_vision: 2 messages, 0 services")

set(MSG_I_FLAGS "-Icolor_gradient_vision:/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(color_gradient_vision_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg" NAME_WE)
add_custom_target(_color_gradient_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "color_gradient_vision" "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg" ""
)

get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPositionPairs.msg" NAME_WE)
add_custom_target(_color_gradient_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "color_gradient_vision" "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPositionPairs.msg" "color_gradient_vision/ColorAndPosition"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(color_gradient_vision
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/color_gradient_vision
)
_generate_msg_cpp(color_gradient_vision
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPositionPairs.msg"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/color_gradient_vision
)

### Generating Services

### Generating Module File
_generate_module_cpp(color_gradient_vision
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/color_gradient_vision
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(color_gradient_vision_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(color_gradient_vision_generate_messages color_gradient_vision_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg" NAME_WE)
add_dependencies(color_gradient_vision_generate_messages_cpp _color_gradient_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPositionPairs.msg" NAME_WE)
add_dependencies(color_gradient_vision_generate_messages_cpp _color_gradient_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(color_gradient_vision_gencpp)
add_dependencies(color_gradient_vision_gencpp color_gradient_vision_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS color_gradient_vision_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(color_gradient_vision
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/color_gradient_vision
)
_generate_msg_eus(color_gradient_vision
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPositionPairs.msg"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/color_gradient_vision
)

### Generating Services

### Generating Module File
_generate_module_eus(color_gradient_vision
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/color_gradient_vision
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(color_gradient_vision_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(color_gradient_vision_generate_messages color_gradient_vision_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg" NAME_WE)
add_dependencies(color_gradient_vision_generate_messages_eus _color_gradient_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPositionPairs.msg" NAME_WE)
add_dependencies(color_gradient_vision_generate_messages_eus _color_gradient_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(color_gradient_vision_geneus)
add_dependencies(color_gradient_vision_geneus color_gradient_vision_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS color_gradient_vision_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(color_gradient_vision
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/color_gradient_vision
)
_generate_msg_lisp(color_gradient_vision
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPositionPairs.msg"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/color_gradient_vision
)

### Generating Services

### Generating Module File
_generate_module_lisp(color_gradient_vision
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/color_gradient_vision
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(color_gradient_vision_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(color_gradient_vision_generate_messages color_gradient_vision_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg" NAME_WE)
add_dependencies(color_gradient_vision_generate_messages_lisp _color_gradient_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPositionPairs.msg" NAME_WE)
add_dependencies(color_gradient_vision_generate_messages_lisp _color_gradient_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(color_gradient_vision_genlisp)
add_dependencies(color_gradient_vision_genlisp color_gradient_vision_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS color_gradient_vision_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(color_gradient_vision
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/color_gradient_vision
)
_generate_msg_nodejs(color_gradient_vision
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPositionPairs.msg"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/color_gradient_vision
)

### Generating Services

### Generating Module File
_generate_module_nodejs(color_gradient_vision
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/color_gradient_vision
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(color_gradient_vision_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(color_gradient_vision_generate_messages color_gradient_vision_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg" NAME_WE)
add_dependencies(color_gradient_vision_generate_messages_nodejs _color_gradient_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPositionPairs.msg" NAME_WE)
add_dependencies(color_gradient_vision_generate_messages_nodejs _color_gradient_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(color_gradient_vision_gennodejs)
add_dependencies(color_gradient_vision_gennodejs color_gradient_vision_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS color_gradient_vision_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(color_gradient_vision
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/color_gradient_vision
)
_generate_msg_py(color_gradient_vision
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPositionPairs.msg"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/color_gradient_vision
)

### Generating Services

### Generating Module File
_generate_module_py(color_gradient_vision
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/color_gradient_vision
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(color_gradient_vision_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(color_gradient_vision_generate_messages color_gradient_vision_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPosition.msg" NAME_WE)
add_dependencies(color_gradient_vision_generate_messages_py _color_gradient_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-acl/project_workspace/src/color_gradient_vision/msg/ColorAndPositionPairs.msg" NAME_WE)
add_dependencies(color_gradient_vision_generate_messages_py _color_gradient_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(color_gradient_vision_genpy)
add_dependencies(color_gradient_vision_genpy color_gradient_vision_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS color_gradient_vision_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/color_gradient_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/color_gradient_vision
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(color_gradient_vision_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(color_gradient_vision_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/color_gradient_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/color_gradient_vision
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(color_gradient_vision_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(color_gradient_vision_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/color_gradient_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/color_gradient_vision
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(color_gradient_vision_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(color_gradient_vision_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/color_gradient_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/color_gradient_vision
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(color_gradient_vision_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(color_gradient_vision_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/color_gradient_vision)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/color_gradient_vision\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/color_gradient_vision
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(color_gradient_vision_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(color_gradient_vision_generate_messages_py std_msgs_generate_messages_py)
endif()
