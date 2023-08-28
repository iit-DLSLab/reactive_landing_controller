cmake_minimum_required(VERSION 2.8.3)
project(landing_controller)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-o3")

# Without this, any build libraries automatically have names "lib{x}.so"
set(CMAKE_SHARED_MODULE_PREFIX "")
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} /opt/openrobots/lib/cmake/eigenpy")

# Find python and Boost - both are required dependencies
find_package(catkin REQUIRED COMPONENTS roscpp )
find_package(PythonLibs 3.8 REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


add_library(SLIP_dynamics_lib MODULE
  controller/SLIP_dynamics/slip_dynamics.cpp
  controller/SLIP_dynamics/slip_dynamics.h
)


target_link_libraries(SLIP_dynamics_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} /opt/openrobots/lib/libeigenpy.so)
target_compile_definitions(SLIP_dynamics_lib PRIVATE SLIP_DYNAMICS_LIBRARY)
target_include_directories(SLIP_dynamics_lib PRIVATE ${PYTHON_INCLUDE_DIRS} /opt/openrobots/include)

install(TARGETS SLIP_dynamics_lib
  DESTINATION DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/lib)
