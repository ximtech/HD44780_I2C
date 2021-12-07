cmake_minimum_required(VERSION 3.20)
project(HD44780_LCD_I2C C)

set(CMAKE_C_STANDARD 99)

include(cmake/CPM.cmake)

#CPMAddPackage(
#        NAME STM32Core
#        GITHUB_REPOSITORY ximtech/STM32Core
#        GIT_TAG origin/main)

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

#add_subdirectory(${STM32_CORE_SOURCE_DIR}/I2C/Polling)
#add_subdirectory(${I2C_Polling_SOURCE_DIR})
#include_directories(${STM32_CORE_SOURCE_DIR}/I2C/Polling)
#link_directories(${STM32_CORE_SOURCE_DIR}/I2C ${I2C_Polling_SOURCE_DIR})

#include_directories(
#        ${HD44780_LCD_I2C_DIRECTORY}
#        )

set(SOURCE_FILES ${PROJECT_NAME}.c include/${PROJECT_NAME}.h)
add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

#target_include_directories(${PROJECT_NAME} PUBLIC
#        ${HD44780_LCD_I2C_DIRECTORY}
#        ${STM32_CORE_SOURCE_DIR}/I2C
#        ${I2C_Polling_SOURCE_DIR}
#        ${I2C_Polling_SOURCE_DIR}/include)
#
#target_link_directories(${PROJECT_NAME} PUBLIC
#        ${STM32_CORE_SOURCE_DIR}
#        ${STM32_CORE_SOURCE_DIR}/I2C/Polling
#        )

#target_include_directories(${PROJECT_NAME} PUBLIC
#        ${STM32_CORE_SOURCE_DIR}/I2C
#        ${STM32_CORE_SOURCE_DIR}/I2C/Polling
#        ${STM32_CORE_SOURCE_DIR}/I2C/Polling/include
#        ${HD44780_LCD_I2C_SOURCE_DIR}
#        ${HD44780_LCD_I2C_SOURCE_DIR}/include)
##
#target_link_libraries(${PROJECT_NAME} STM32_CORE I2C_Polling)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}.h
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Export
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})

install(EXPORT ${PROJECT_NAME}Export
        FILE ${PROJECT_NAME}-config.cmake
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})

