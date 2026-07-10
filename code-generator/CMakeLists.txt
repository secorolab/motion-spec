cmake_minimum_required(VERSION 3.10)
project(ral_demo)

find_package(robif2b)
find_package(hddc2b)
find_package(Eigen3)
find_package(orocos_kdl)
find_package(urdfdom_headers)
find_package(urdfdom)
# Optional: present only when monitor events are wired to a coord2b FSM.
find_package(coord2b)
find_package(Threads)

option(MOTION_SPEC_ENABLE_INTROSPECTION "Enable generated runtime introspection logging" OFF)

add_executable(main
  ref_main.cpp
)

target_include_directories(main
  PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    headers
    ${coord2b_INCLUDE_DIRS}
)

target_link_libraries(main
  robif2b::ethercat
  robif2b::kelo
  robif2b::kinova_gen3
  hddc2b::hddc2b
  orocos-kdl
  urdfdom_headers::urdfdom_headers
  urdfdom::urdf_parser
  kdl_parser
  ${coord2b_LIBRARIES}
)

if(MOTION_SPEC_ENABLE_INTROSPECTION)
  target_compile_definitions(main PRIVATE MOTION_SPEC_ENABLE_INTROSPECTION)
  target_link_libraries(main Threads::Threads)
endif()

target_compile_options(main
  PRIVATE
    -Wall
    -Wextra
)
