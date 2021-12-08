# HD44780_I2C

**STM32** LL(Low Layer) library for I2C LCD. Works for both 16x2 LCD and 20x4 LCD etc.

### LCD 16x2

<img src="https://github.com/ximtech/HD44780_I2C/blob/main/example/LCD16x2.PNG" alt="image" width="300"/>

### LCD 20x4

<img src="https://github.com/ximtech/HD44780_I2C/blob/main/example/LCD20x4.PNG" alt="image" width="300"/>

### Add as CPM project dependency

How to add CPM to the project, check the [link](https://github.com/cpm-cmake/CPM.cmake)

- Add STM32Core package dependency: [link](https://github.com/ximtech/STM32Core)

```cmake
CPMAddPackage(
        NAME HD44780_I2C
        GITHUB_REPOSITORY ximtech/HD44780_I2C
        GIT_TAG origin/main)
```

### Project configuration

1. Start project with STM32CubeMX:
    * [I2C configuration](https://github.com/ximtech/HD44780_I2C/blob/main/example/config.PNG)
2. Select: Project Manager -> Advanced Settings -> I2C -> LL
3. Generate Code
4. Add sources to project:

```cmake
include_directories(${includes} ${HD44780_LCD_I2C_DIRECTORY})   # source directories
file(GLOB_RECURSE SOURCES ${sources} ${HD44780_LCD_I2C_SOURCES})    # source files

add_subdirectory(${STM32_CORE_SOURCE_DIR}/I2C/Polling)  # add I2C dependency, core package must be included before
add_subdirectory(${HD44780_LCD_I2C_SOURCE_DIR})
```

3. Then Build -> Clean -> Rebuild Project

## Wiring

- PCF8574 port extender pinout:\
- <img src="https://github.com/ximtech/HD44780_I2C/blob/main/example/PCF8574_pinout.PNG" alt="image" width="300"/>
- <img src="https://github.com/ximtech/HD44780_I2C/blob/main/example/soldering_view.PNG" alt="image" width="300"/>
  
## Usage
In `HD44780_LCD_I2C.h` default defines. Override them in `main.h` if needed
```c
#define HD44780_LCD_COL_COUNT 20
#define HD44780_LCD_ROW_COUNT 4
#define HD44780_LCD_I2C_DEVICE_ADDRESS 0x4e // default PCF8574 address
```
- If address differs, use scan mode: [example](https://github.com/ximtech/STM32Core/blob/main/I2C/Polling/example/example.c)
- Usage example: [link](https://github.com/ximtech/HD44780_I2C/blob/main/example/example.c)
