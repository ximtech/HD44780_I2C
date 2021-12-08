cmake_minimum_required(VERSION 3.20)
project(HD44780_LCD_I2C C)

set(CMAKE_C_STANDARD 99)

include(cmake/CPM.cmake)

set(HD44780_LCD_I2C_DIRECTORY
        ${I2C_POLLING_DIRECTORY}
        ${HD44780_LCD_I2C_SOURCE_DIR}
        ${HD44780_LCD_I2C_SOURCE_DIR}/include
        CACHE STRING "HD44780 LCD I2C directories include to the main project" FORCE)

set(HD44780_LCD_I2C_SOURCES
        ${I2C_POLLING_SOURCES}
        ${HD44780_LCD_I2C_SOURCE_DIR}/${PROJECT_NAME}.c
        ${HD44780_LCD_I2C_SOURCE_DIR}/include/${PROJECT_NAME}.h
        CACHE STRING "HD44780 LCD I2C source files include to the main project" FORCE)

link_directories(${I2C_POLLING_DIRECTORY})
link_libraries(I2C_Polling)

include_directories(${HD44780_LCD_I2C_DIRECTORY})