cmake_minimum_required(VERSION 3.5)
project(go1_hal_test)

find_package(go1hal QUIET)

add_executable(test_go1_hal
	test_go1_hal.cpp
)

target_link_libraries(test_go1_hal
	# PRIVATE
	PUBLIC
		go1hal
)

set_property(
  TARGET test_go1_hal
  PROPERTY
    CXX_STANDARD 11
)

# TODO: Implement the test for python
