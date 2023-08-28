# References:
# https://stackoverflow.com/questions/56135785/correctly-set-the-location-of-imported-cmake-targets-for-an-installed-package

cmake_minimum_required(VERSION 3.5)
project(go1_hal)

# Ensures CMake creates the file compile_commands.json (Required for Vim-YCM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_FIND_DEBUG_MODE FALSE)

set(VERSION 0.1.0)

option(BUILD_GO1HAL_TEST "Build the tests" ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(INCLUDE_INSTALL_DIR include)  # include
# set(LIBRARY_INSTALL_DIR lib/go1hal)
set(LIBRARY_INSTALL_DIR lib)
set(ConfigFileInstallDir lib/cmake/go1hal)


# ###################
# GO1 HAL LIBRARY
# ###################

# Unitree-SDK dependency
find_package(unitree_legged_sdk QUIET)
if(NOT unitree_legged_sdk_FOUND)
 # Compile and install unitreesdk if it is not already installed
 add_subdirectory(third_party/unitree_legged_sdk)
endif()

# add_library(go1hal STATIC  # TODO: decide if want it static or dynamic
add_library(go1hal SHARED
  src/go1_hal.cpp
)

target_link_libraries(go1hal
  PUBLIC
  unitree_legged_sdk
)

target_include_directories(go1hal
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>   # <prefix>/include
    # $<INSTALL_INTERFACE:$<$<CONFIG:Debug>:$<INSTALL_PREFIX>/include>>
)

set_property(TARGET go1hal PROPERTY CXX_STANDARD 11)  # At least std c++11

# # TODO: this is not necesary when using a shared library. Uncomment it if we decide to make the library static
# set_property(TARGET go1hal PROPERTY POSITION_INDEPENDENT_CODE ON)


# ##############
# PY GO1 HAL
# ##############

# Pybind11 dependency
add_subdirectory(third_party/pybind11)

pybind11_add_module(pygo1_hal
  src/python_interface.cpp
)
target_link_libraries(pygo1_hal
  PUBLIC
    go1hal
    pybind11::embed
)


# #####
# TESTS
# #####
if(BUILD_GO1HAL_TEST)
  add_subdirectory(test)
endif()
 
 
# #######
# INSTALL
# #######
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/include/
  DESTINATION include
)
install(TARGETS
  go1hal
  EXPORT go1halTargets
  RUNTIME DESTINATION bin COMPONENT Runtime
  LIBRARY DESTINATION lib COMPONENT Runtime
  ARCHIVE DESTINATION lib COMPONENT Development
  PUBLIC_HEADER DESTINATION include COMPONENT Development
  BUNDLE DESTINATION bin COMPONENT Runtime  
)
# Get python-site directory. Ref: https://stackoverflow.com/questions/1242904/finding-python-site-packages-directory-with-cmake
if(NOT PYTHON_SITE)
  # execute_process(
  #   COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
  #     from distutils import sysconfig as sc
  #     print(sc.get_python_lib(prefix='', plat_specific=True))"
  #   OUTPUT_VARIABLE PYTHON_SITE
  #   OUTPUT_STRIP_TRAILING_WHITESPACE
  # )
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
      from distutils import sysconfig as sc
      print(sc.get_python_lib(prefix='', standard_lib=True, plat_specific=True))"
      # print(sc.get_python_lib(prefix='', standard_lib=True, plat_specific=True)+'/dist-packages')"
    OUTPUT_VARIABLE PYTHON_SITE
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif()
install(TARGETS
  pygo1_hal
  COMPONENT python
  LIBRARY DESTINATION "${PYTHON_SITE}"
)
unset(PYTHON_SITE CACHE)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/go1halConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/go1halConfig.cmake"
  INSTALL_DESTINATION "${ConfigFileInstallDir}"
  PATH_VARS INCLUDE_INSTALL_DIR LIBRARY_INSTALL_DIR
)
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/go1halConfigVersion.cmake"
  VERSION ${VERSION}
  COMPATIBILITY AnyNewerVersion
  # COMPATIBILITY ExactVersion
)

# export( # Creates Targets.cmake for build tree
#   EXPORT go1halTargets
#   FILE "${PROJECT_BINARY_DIR}/go1halTargets.cmake"
# )
# install( # Creates Targets.cmake referencing the relocatable install location
#   EXPORT go1halTargets
#   FILE go1halTargets.cmake
#   # DESTINATION lib/cmake  # TODO: Is it ok this? or do we need
#   DESTINATION "${ConfigFileInstallDir}"
# )
# install(FILES
#   "${PROJECT_BINARY_DIR}/go1halConfig.cmake"
#   "${PROJECT_BINARY_DIR}/go1halConfigVersion.cmake"
#   # "${PROJECT_BINARY_DIR}/unitreesdkTargets.cmake"
#   DESTINATION "${ConfigFileInstallDir}"
# )


