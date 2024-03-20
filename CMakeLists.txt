cmake_minimum_required(VERSION 3.22...3.27)

project(ICM20649)

set(LIBRARY ${PROJECT_NAME})

enable_language(CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${LIBRARY} STATIC
	Src/ICM20649.c
)

set(CMAKE_CROSSCOMPILING FALSE) #Make this into a command line argument

target_include_directories(${LIBRARY} PUBLIC
	Inc
)

include(stm32l4cube.cmake)
target_link_libraries(${LIBRARY} PUBLIC STM32CUBEL4_HAL)

if(NOT CMAKE_CROSSCOMPILING) 

	include(FetchContent)
	FetchContent_Declare(
	  googletest
	  GIT_REPOSITORY https://github.com/google/googletest
	  GIT_TAG        origin/main
	)
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
	FetchContent_MakeAvailable(googletest)
	enable_testing()

	set(TEST_EXECUTABLE ${PROJECT_NAME}_Test)

	add_executable(${TEST_EXECUTABLE}
		Test/ICM20649.test.cpp
	)

	target_include_directories(${TEST_EXECUTABLE} PUBLIC
		Inc
	)

	target_link_libraries(${TEST_EXECUTABLE} PUBLIC
		${LIBRARY}
		GTest::gtest_main
		GTest::gmock
	)

	include(GoogleTest)
	gtest_discover_tests(${TEST_EXECUTABLE})
endif()