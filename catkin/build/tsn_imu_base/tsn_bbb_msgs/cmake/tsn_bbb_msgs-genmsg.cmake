# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tsn_bbb_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Itsn_bbb_msgs:/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tsn_bbb_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/Voltage.msg" NAME_WE)
add_custom_target(_tsn_bbb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tsn_bbb_msgs" "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/Voltage.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/GPIOOut.msg" NAME_WE)
add_custom_target(_tsn_bbb_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tsn_bbb_msgs" "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/GPIOOut.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tsn_bbb_msgs
  "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/Voltage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsn_bbb_msgs
)
_generate_msg_cpp(tsn_bbb_msgs
  "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/GPIOOut.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsn_bbb_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(tsn_bbb_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsn_bbb_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tsn_bbb_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tsn_bbb_msgs_generate_messages tsn_bbb_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/Voltage.msg" NAME_WE)
add_dependencies(tsn_bbb_msgs_generate_messages_cpp _tsn_bbb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/GPIOOut.msg" NAME_WE)
add_dependencies(tsn_bbb_msgs_generate_messages_cpp _tsn_bbb_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tsn_bbb_msgs_gencpp)
add_dependencies(tsn_bbb_msgs_gencpp tsn_bbb_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tsn_bbb_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tsn_bbb_msgs
  "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/Voltage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsn_bbb_msgs
)
_generate_msg_lisp(tsn_bbb_msgs
  "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/GPIOOut.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsn_bbb_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(tsn_bbb_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsn_bbb_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tsn_bbb_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tsn_bbb_msgs_generate_messages tsn_bbb_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/Voltage.msg" NAME_WE)
add_dependencies(tsn_bbb_msgs_generate_messages_lisp _tsn_bbb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/GPIOOut.msg" NAME_WE)
add_dependencies(tsn_bbb_msgs_generate_messages_lisp _tsn_bbb_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tsn_bbb_msgs_genlisp)
add_dependencies(tsn_bbb_msgs_genlisp tsn_bbb_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tsn_bbb_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tsn_bbb_msgs
  "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/Voltage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsn_bbb_msgs
)
_generate_msg_py(tsn_bbb_msgs
  "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/GPIOOut.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsn_bbb_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(tsn_bbb_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsn_bbb_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tsn_bbb_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tsn_bbb_msgs_generate_messages tsn_bbb_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/Voltage.msg" NAME_WE)
add_dependencies(tsn_bbb_msgs_generate_messages_py _tsn_bbb_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/ros_workspace/tsn_senior_project/catkin/src/tsn_imu_base/tsn_bbb_msgs/msg/GPIOOut.msg" NAME_WE)
add_dependencies(tsn_bbb_msgs_generate_messages_py _tsn_bbb_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tsn_bbb_msgs_genpy)
add_dependencies(tsn_bbb_msgs_genpy tsn_bbb_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tsn_bbb_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsn_bbb_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsn_bbb_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(tsn_bbb_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsn_bbb_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsn_bbb_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(tsn_bbb_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsn_bbb_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsn_bbb_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsn_bbb_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(tsn_bbb_msgs_generate_messages_py std_msgs_generate_messages_py)
