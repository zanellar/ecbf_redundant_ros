cmake_minimum_required(VERSION 3.4)
project(ecbf_franka_controller)

# ---------------------------------------------------------------------------
# Compiler and dependency configuration
# ---------------------------------------------------------------------------

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  controller_manager_msgs
  franka_hw
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  std_msgs
)
find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)

# ---------------------------------------------------------------------------
# Diagnostic message generation
# ---------------------------------------------------------------------------

add_message_files(FILES EcbfDiagnostics.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ecbf_franka_controller
  CATKIN_DEPENDS controller_interface controller_manager controller_manager_msgs franka_hw hardware_interface message_runtime pluginlib realtime_tools roscpp rospy std_msgs
  DEPENDS Franka
)

# ---------------------------------------------------------------------------
# Real-time controller library
# ---------------------------------------------------------------------------

add_library(ecbf_franka_controller
  src/cbf.cpp
  src/ecbf_controller.cpp
  src/math_utils.cpp
)
add_dependencies(ecbf_franka_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(ecbf_franka_controller SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS} ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_include_directories(ecbf_franka_controller PUBLIC include)
target_link_libraries(ecbf_franka_controller PUBLIC ${Franka_LIBRARIES} ${catkin_LIBRARIES})

# ---------------------------------------------------------------------------
# Non-real-time helper scripts
# ---------------------------------------------------------------------------

catkin_install_python(
  PROGRAMS scripts/diagnostics_csv_logger.py scripts/trial_supervisor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------

install(TARGETS ecbf_franka_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES ecbf_franka_controller_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# ---------------------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------------------

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_cbf test/test_cbf.cpp src/cbf.cpp src/math_utils.cpp)
  if(TARGET test_cbf)
    add_dependencies(test_cbf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_include_directories(test_cbf SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
    target_include_directories(test_cbf PRIVATE include)
    target_link_libraries(test_cbf ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_math_utils test/test_math_utils.cpp src/math_utils.cpp)
  if(TARGET test_math_utils)
    add_dependencies(test_math_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_include_directories(test_math_utils SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
    target_include_directories(test_math_utils PRIVATE include)
    target_link_libraries(test_math_utils ${catkin_LIBRARIES})
  endif()
endif()
