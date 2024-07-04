# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "navigation2: 7 messages, 0 services")

set(MSG_I_FLAGS "-Inavigation2:/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(navigation2_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Goal.msg" NAME_WE)
add_custom_target(_navigation2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation2" "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Goal.msg" ""
)

get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Planner_state.msg" NAME_WE)
add_custom_target(_navigation2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation2" "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Planner_state.msg" ""
)

get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Enc_dist.msg" NAME_WE)
add_custom_target(_navigation2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation2" "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Enc_dist.msg" ""
)

get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/gps_data.msg" NAME_WE)
add_custom_target(_navigation2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation2" "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/gps_data.msg" ""
)

get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/enc_feed.msg" NAME_WE)
add_custom_target(_navigation2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation2" "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/enc_feed.msg" ""
)

get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/imu_angle.msg" NAME_WE)
add_custom_target(_navigation2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation2" "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/imu_angle.msg" ""
)

get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/auto.msg" NAME_WE)
add_custom_target(_navigation2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation2" "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/auto.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation2
)
_generate_msg_cpp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Planner_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation2
)
_generate_msg_cpp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Enc_dist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation2
)
_generate_msg_cpp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/gps_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation2
)
_generate_msg_cpp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/enc_feed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation2
)
_generate_msg_cpp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/imu_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation2
)
_generate_msg_cpp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/auto.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation2
)

### Generating Services

### Generating Module File
_generate_module_cpp(navigation2
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation2
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(navigation2_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(navigation2_generate_messages navigation2_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Goal.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_cpp _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Planner_state.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_cpp _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Enc_dist.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_cpp _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/gps_data.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_cpp _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/enc_feed.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_cpp _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/imu_angle.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_cpp _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/auto.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_cpp _navigation2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navigation2_gencpp)
add_dependencies(navigation2_gencpp navigation2_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation2_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation2
)
_generate_msg_eus(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Planner_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation2
)
_generate_msg_eus(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Enc_dist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation2
)
_generate_msg_eus(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/gps_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation2
)
_generate_msg_eus(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/enc_feed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation2
)
_generate_msg_eus(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/imu_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation2
)
_generate_msg_eus(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/auto.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation2
)

### Generating Services

### Generating Module File
_generate_module_eus(navigation2
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation2
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(navigation2_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(navigation2_generate_messages navigation2_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Goal.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_eus _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Planner_state.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_eus _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Enc_dist.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_eus _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/gps_data.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_eus _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/enc_feed.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_eus _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/imu_angle.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_eus _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/auto.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_eus _navigation2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navigation2_geneus)
add_dependencies(navigation2_geneus navigation2_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation2_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation2
)
_generate_msg_lisp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Planner_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation2
)
_generate_msg_lisp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Enc_dist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation2
)
_generate_msg_lisp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/gps_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation2
)
_generate_msg_lisp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/enc_feed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation2
)
_generate_msg_lisp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/imu_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation2
)
_generate_msg_lisp(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/auto.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation2
)

### Generating Services

### Generating Module File
_generate_module_lisp(navigation2
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation2
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(navigation2_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(navigation2_generate_messages navigation2_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Goal.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_lisp _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Planner_state.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_lisp _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Enc_dist.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_lisp _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/gps_data.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_lisp _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/enc_feed.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_lisp _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/imu_angle.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_lisp _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/auto.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_lisp _navigation2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navigation2_genlisp)
add_dependencies(navigation2_genlisp navigation2_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation2_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation2
)
_generate_msg_nodejs(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Planner_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation2
)
_generate_msg_nodejs(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Enc_dist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation2
)
_generate_msg_nodejs(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/gps_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation2
)
_generate_msg_nodejs(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/enc_feed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation2
)
_generate_msg_nodejs(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/imu_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation2
)
_generate_msg_nodejs(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/auto.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation2
)

### Generating Services

### Generating Module File
_generate_module_nodejs(navigation2
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation2
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(navigation2_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(navigation2_generate_messages navigation2_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Goal.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_nodejs _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Planner_state.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_nodejs _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Enc_dist.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_nodejs _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/gps_data.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_nodejs _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/enc_feed.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_nodejs _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/imu_angle.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_nodejs _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/auto.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_nodejs _navigation2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navigation2_gennodejs)
add_dependencies(navigation2_gennodejs navigation2_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation2_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation2
)
_generate_msg_py(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Planner_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation2
)
_generate_msg_py(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Enc_dist.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation2
)
_generate_msg_py(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/gps_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation2
)
_generate_msg_py(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/enc_feed.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation2
)
_generate_msg_py(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/imu_angle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation2
)
_generate_msg_py(navigation2
  "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/auto.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation2
)

### Generating Services

### Generating Module File
_generate_module_py(navigation2
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation2
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(navigation2_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(navigation2_generate_messages navigation2_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Goal.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_py _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Planner_state.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_py _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/Enc_dist.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_py _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/gps_data.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_py _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/enc_feed.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_py _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/imu_angle.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_py _navigation2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/nvidia/galileo/home/nvidia/galileo2024/src/navigation2/msg/auto.msg" NAME_WE)
add_dependencies(navigation2_generate_messages_py _navigation2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navigation2_genpy)
add_dependencies(navigation2_genpy navigation2_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation2_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation2
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(navigation2_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation2
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(navigation2_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation2
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(navigation2_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation2
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(navigation2_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation2)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation2\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation2
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(navigation2_generate_messages_py std_msgs_generate_messages_py)
endif()
