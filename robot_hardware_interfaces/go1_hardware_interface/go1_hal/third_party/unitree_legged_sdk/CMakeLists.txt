cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

include_directories(include)

add_compile_options(-std=c++14)

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

link_directories(lib/cpp/${ARCH})
set(EXTRA_LIBS -pthread libunitree_legged_sdk.a)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

# one pc one process
add_executable(example_position example/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

add_executable(example_velocity example/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque example/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk example/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

add_executable(example_joystick example/example_joystick.cpp)
target_link_libraries(example_joystick ${EXTRA_LIBS})


# install
install(TARGETS
  example_position example_velocity example_torque example_walk example_joystick
  DESTINATION bin/unitree)
install(DIRECTORY lib/cpp/${ARCH}/
  DESTINATION lib/unitree
  USE_SOURCE_PERMISSIONS)


#######
#Domingo stuff create dynamic library from static
#######


set(INCLUDE_INSTALL_DIR include)  # include
set(LIBRARY_INSTALL_DIR lib)
set(ConfigFileInstallDir lib/cmake/unitree_legged_sdk)
add_library(unitree_legged_sdk INTERFACE)

target_include_directories(unitree_legged_sdk
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>  
)

target_link_libraries(unitree_legged_sdk
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.a>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${LIBRARY_INSTALL_DIR}/libunitree_legged_sdk.a>
    lcm
)

# #######
# INSTALL
# #######
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/include/
  DESTINATION include
)
install(FILES
  ${PROJECT_SOURCE_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.a
  DESTINATION ${LIBRARY_INSTALL_DIR}
)
install(TARGETS unitree_legged_sdk
EXPORT unitreesdkTargets
RUNTIME DESTINATION bin COMPONENT Runtime
LIBRARY DESTINATION lib COMPONENT Runtime
ARCHIVE DESTINATION lib COMPONENT Development
PUBLIC_HEADER DESTINATION include COMPONENT Development
BUNDLE DESTINATION bin COMPONENT Runtime
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
"${PROJECT_SOURCE_DIR}/cmake/unitree_legged_sdkConfig.cmake.in"
"${PROJECT_BINARY_DIR}/unitree_legged_sdkConfig.cmake"
INSTALL_DESTINATION "${ConfigFileInstallDir}"
PATH_VARS INCLUDE_INSTALL_DIR LIBRARY_INSTALL_DIR
)



