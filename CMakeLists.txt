cmake_minimum_required(VERSION 3.22...3.27)

project(ICM20649)

set(TARGET_EXECUTABLE ${CMAKE_PROJECT_NAME})

enable_language(C CXX ASM)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

include(stm32l4cube.cmake)

if(CMAKE_CROSSCOMPILING)
	include(CMake/scripts.cmake)
	include(CMake/mcu.cmake)

	file(GLOB_RECURSE APPLICATION_SOURCES
		${CMAKE_SOURCE_DIR}/Src/*.c
	)

	add_executable(${TARGET_EXECUTABLE}
		${APPLICATION_SOURCES}
		${STARTUP_SCRIPT}
	)

	target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
	Inc
)

	set(CROSSCOMPILE_C_OPTIONS
		-fdata-sections
		-ffunction-sections
	)
	set(CROSSCOMPILE_CXX_OPTIONS
		-fno-rtti
		-fno-exceptions
	)

	set(CROSSCOMPILE_OPTIMISATION_OPTIONS_DEBUG
		-Og
		-ggdb3
	)
	set(CROSSCOMPILE_OPTIMISATION_OPTIONS_RELWITHDEBINFO
		-O3
		-ggdb
	)

	add_compile_options(
		${CPU_COMPILE_OPTIONS}
		${CROSSCOMPILE_C_OPTIONS}
		"$<$<COMPILE_LANGUAGE:CXX>:${CROSSCOMPILE_CXX_OPTIONS}>"
		"$<$<CONFIG:Debug>:${CROSSCOMPILE_OPTIMISATION_OPTIONS_DEBUG}>"
		"$<$<CONFIG:RelWithDebInfo>:${CROSSCOMPILE_OPTIMISATION_OPTIONS_RELWITHDEBINFO}>"
	)

	target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC STM32CUBEL4_HAL)

	add_compile_definitions(
		$<$<CONFIG:Debug>:DEBUG>
	)

	add_link_options(
		${CPU_COMPILE_OPTIONS}
		--specs=nano.specs
		-T${LINKER_SCRIPT}

		LINKER:--start-group
		-lc
		-lm
		-lnosys
		LINKER:--end-group

		LINKER:--gc-sections
		LINKER:--print-memory-usage
		#LINKER:--no-warn-rwx-segment # Workaround — Something isn't right in ST's startup code.
	)
endif()

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

	# Incomplete - need to change framework to FFF from GTest to stub c functions 
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