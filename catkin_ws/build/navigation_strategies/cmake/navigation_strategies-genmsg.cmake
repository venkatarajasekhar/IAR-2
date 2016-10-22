# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "navigation_strategies: 1 messages, 0 services")

set(MSG_I_FLAGS "-Inavigation_strategies:/home/viki/catkin_ws/src/navigation_strategies/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(navigation_strategies_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(navigation_strategies
  "/home/viki/catkin_ws/src/navigation_strategies/msg/DirDistrib.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation_strategies
)

### Generating Services

### Generating Module File
_generate_module_cpp(navigation_strategies
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation_strategies
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(navigation_strategies_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(navigation_strategies_generate_messages navigation_strategies_generate_messages_cpp)

# target for backward compatibility
add_custom_target(navigation_strategies_gencpp)
add_dependencies(navigation_strategies_gencpp navigation_strategies_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation_strategies_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(navigation_strategies
  "/home/viki/catkin_ws/src/navigation_strategies/msg/DirDistrib.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation_strategies
)

### Generating Services

### Generating Module File
_generate_module_lisp(navigation_strategies
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation_strategies
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(navigation_strategies_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(navigation_strategies_generate_messages navigation_strategies_generate_messages_lisp)

# target for backward compatibility
add_custom_target(navigation_strategies_genlisp)
add_dependencies(navigation_strategies_genlisp navigation_strategies_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation_strategies_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(navigation_strategies
  "/home/viki/catkin_ws/src/navigation_strategies/msg/DirDistrib.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_strategies
)

### Generating Services

### Generating Module File
_generate_module_py(navigation_strategies
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_strategies
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(navigation_strategies_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(navigation_strategies_generate_messages navigation_strategies_generate_messages_py)

# target for backward compatibility
add_custom_target(navigation_strategies_genpy)
add_dependencies(navigation_strategies_genpy navigation_strategies_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation_strategies_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation_strategies)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation_strategies
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(navigation_strategies_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation_strategies)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation_strategies
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(navigation_strategies_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_strategies)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_strategies\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_strategies
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(navigation_strategies_generate_messages_py std_msgs_generate_messages_py)
