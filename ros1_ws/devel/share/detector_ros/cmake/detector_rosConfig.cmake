# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(detector_ros_CONFIG_INCLUDED)
  return()
endif()
set(detector_ros_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(detector_ros_SOURCE_PREFIX /home/skylar/Penn/PennAiR/ros1_ws/src/detector_ros)
  set(detector_ros_DEVEL_PREFIX /home/skylar/Penn/PennAiR/ros1_ws/devel)
  set(detector_ros_INSTALL_PREFIX "")
  set(detector_ros_PREFIX ${detector_ros_DEVEL_PREFIX})
else()
  set(detector_ros_SOURCE_PREFIX "")
  set(detector_ros_DEVEL_PREFIX "")
  set(detector_ros_INSTALL_PREFIX /home/skylar/Penn/PennAiR/ros1_ws/install)
  set(detector_ros_PREFIX ${detector_ros_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'detector_ros' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(detector_ros_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/skylar/Penn/PennAiR/ros1_ws/devel/include " STREQUAL " ")
  set(detector_ros_INCLUDE_DIRS "")
  set(_include_dirs "/home/skylar/Penn/PennAiR/ros1_ws/devel/include")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'skylar <skylar@todo.todo>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${detector_ros_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'detector_ros' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'detector_ros' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/skylar/Penn/PennAiR/ros1_ws/src/detector_ros/${idir}'.  ${_report}")
    endif()
    _list_append_unique(detector_ros_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, generator expressions, target names, and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND detector_ros_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND detector_ros_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT detector_ros_NUM_DUMMY_TARGETS)
      set(detector_ros_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::detector_ros::wrapped-linker-option${detector_ros_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR detector_ros_NUM_DUMMY_TARGETS "${detector_ros_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::detector_ros::wrapped-linker-option${detector_ros_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND detector_ros_LIBRARIES "${interface_target_name}")
  elseif(${library} MATCHES "^\\$<")
    list(APPEND detector_ros_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND detector_ros_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND detector_ros_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/skylar/Penn/PennAiR/ros1_ws/devel/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/python_orocos_kdl/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/panda_simulator_examples/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/panda_simulator/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/panda_sim_moveit/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/panda_gazebo/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/panda_sim_custom_action_server/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/panda_sim_controllers/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_tools/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_moveit/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/panda_moveit_config/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/panda_hardware_interface/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/orocos_kinematics_dynamics/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/meam520_labs/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_visualization/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_ros_interface/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_ros_controllers/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_ros/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_panda_description/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_interface/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_gazebo/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_example_controllers/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_control/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_hw/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_core_msgs/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_msgs/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_gripper/lib;/home/skylar/Penn/MEAM5200/meam520_ws/devel_isolated/franka_description/lib;/opt/ros/noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(detector_ros_LIBRARY_DIRS ${lib_path})
      list(APPEND detector_ros_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'detector_ros'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND detector_ros_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(detector_ros_EXPORTED_TARGETS "detector_ros_generate_messages_cpp;detector_ros_generate_messages_eus;detector_ros_generate_messages_lisp;detector_ros_generate_messages_nodejs;detector_ros_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${detector_ros_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "cv_bridge;message_runtime;rospy;sensor_msgs;std_msgs")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 detector_ros_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${detector_ros_dep}_FOUND)
      find_package(${detector_ros_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${detector_ros_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(detector_ros_INCLUDE_DIRS ${${detector_ros_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(detector_ros_LIBRARIES ${detector_ros_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${detector_ros_dep}_LIBRARIES})
  _list_append_deduplicate(detector_ros_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(detector_ros_LIBRARIES ${detector_ros_LIBRARIES})

  _list_append_unique(detector_ros_LIBRARY_DIRS ${${detector_ros_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(detector_ros_EXPORTED_TARGETS ${${detector_ros_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "detector_ros-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${detector_ros_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
