cmake_minimum_required(VERSION 2.6)
project(contact_control)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Include project headers
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

## Boost
# add iostreams for gnuplot-iostream
find_package(Boost 1.46 COMPONENTS system iostreams REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# -----------------------------
# Function: add_programs
# -----------------------------
function(add_programs)
  foreach(prog ${ARGV})
    add_executable(${prog} src/${prog}.cpp)
    target_link_libraries(${prog}
      ${BARRETT_LIBRARIES}
      ${CURSES_LIBRARIES}
    )
  endforeach()
endfunction()

# -----------------------------
# Executables (run on WAM)
# -----------------------------
add_programs(
  hybrid_control
  hybrid_control_ft
)

# Installed libbarrett/libconfig headers use pre-C++17 exception specifications.
set_property(TARGET hybrid_control hybrid_control_ft APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14")
set_property(TARGET hybrid_control_ft APPEND PROPERTY COMPILE_DEFINITIONS
  HYBRID_CONTROL_FT_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/hybrid_control_ft.conf")

option(CONTACT_CONTROL_BUILD_TESTS "Build hardware-free controller tests" OFF)
if(CONTACT_CONTROL_BUILD_TESTS)
  enable_testing()
  add_executable(planar_joint_torque_test tests/planar_joint_torque.cpp)
  target_link_libraries(planar_joint_torque_test ${BARRETT_LIBRARIES})
  set_property(TARGET planar_joint_torque_test APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14 -UNDEBUG")
  add_test(NAME planar_joint_torque COMMAND planar_joint_torque_test)
  add_executable(controller_config_test tests/controller_config.cpp)
  target_link_libraries(controller_config_test ${BARRETT_LIBRARIES})
  set_property(TARGET controller_config_test APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14")
  add_test(NAME controller_config COMMAND controller_config_test
    "${CMAKE_CURRENT_SOURCE_DIR}/config/hybrid_control_ft.conf")
endif()

# Plotting-only executable
# add_executable(compare_torque src/compare_torque.cpp)
# target_link_libraries(compare_torque ${Boost_LIBRARIES})
