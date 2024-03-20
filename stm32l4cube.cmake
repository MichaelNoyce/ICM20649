# Configures the STM32CubeL4 HAL and FATFS libraries.
# It is expected the https://github.com/STMicroelectronics/STM32CubeL4 is cloned to ${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4.

# Generates static libraries:
# - SHARC::HAL
# - SHARC::FATFS
set(STM32CUBEL4_HAL_INCLUDE_DIRECTORIES
	${CMAKE_SOURCE_DIR}/stm32l4xx_hal_driver/Inc
	${CMAKE_SOURCE_DIR}/stm32l4xx_hal_driver/Inc/Legacy
)

file(GLOB STM32CUBEL4_HAL_SOURCES
	${CMAKE_SOURCE_DIR}/stm32l4xx_hal_driver/Src/stm32l4xx_hal_i2c.c
	${CMAKE_SOURCE_DIR}/stm32l4xx_hal_driver/Src/stm32l4xx_hal_i2c_ex.c
	${CMAKE_SOURCE_DIR}/stm32l4xx_hal_driver/Src/stm32l4xx_hal_gpio.c
	${CMAKE_SOURCE_DIR}/stm32l4xx_hal_driver/Src/stm32l4xx_hal_uart.c
	${CMAKE_SOURCE_DIR}/stm32l4xx_hal_driver/Src/stm32l4xx_hal_uart_ex.c
	${CMAKE_SOURCE_DIR}/stm32l4xx_hal_driver/Src/stm32l4xx_hal.c
)

# Workaround - Broken template files should not be compiled.
list(FILTER STM32CUBEL4_HAL_SOURCES EXCLUDE REGEX ".*_template.c")

# HAL Library
add_library(STM32CUBEL4_HAL STATIC
	${STM32CUBEL4_HAL_SOURCES}
)

set(STM32CUBEL4_HAL_COMPILE_DEFINITIONS
	USE_HAL_DRIVER
	STM32L4R5xx
	ARM_MATH_CM4
)

target_compile_definitions(STM32CUBEL4_HAL PUBLIC
	${STM32CUBEL4_HAL_COMPILE_DEFINITIONS}
)

target_include_directories(STM32CUBEL4_HAL SYSTEM
	PUBLIC ${STM32CUBEL4_HAL_INCLUDE_DIRECTORIES}
)

add_library(SHARC::HAL ALIAS STM32CUBEL4_HAL)