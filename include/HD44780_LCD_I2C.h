#pragma once

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "I2C_Polling.h"

#define LCD_COL_COUNT 20
#define LCD_ROW_COUNT 4

#define HD44780_LCD_I2C_DEVICE_ADDRESS 0x4e
#define HD44780_LCD_I2C_TIMEOUT_MS 1000

// Basic commands
#define LCD_CLEAR_DISPLAY 0x01            // Clear the display screen. Execution Time: 1.64ms
#define LCD_SHIFT_CURSOR_RIGHT 0x06        // Shift the cursor right (e.g. data gets written in an incrementing order, left to right)

#define LCD_DISPLAY_ON_CURSOR_OFF 0x0C
#define LCD_DISPLAY_ON_CURSOR_BLINKING 0x0E

#define LCD_MOVE_CURSOR_AT_FIRST_LINE_BEGINNING 0x80    // Force the cursor to the beginning of the 1st line
#define LCD_MOVE_CURSOR_AT_SECOND_LINE_BEGINNING 0xC0    // Force the cursor to the beginning of the 2nd line

#define LCD_SHIFT_POSITION_LEFT 0x10                    // Shift cursor position to the left
#define LCD_SHIFT_POSITION_RIGHT 0x14                    // Shift cursor position to the right
#define LCD_SHIFT_DISPLAY_LEFT 0x18                        // Shift entire display to the left
#define LCD_SHIFT_DISPLAY_RIGHT 0x1C                    // Shift entire display to the right

#define LCD_TWO_LINES_5X8_MATRIX_8BIT_MODE 0x38
#define LCD_TWO_LINES_5X8_MATRIX_4BIT_MODE 0x28
#define LCD_ONE_LINE_8BIT_MODE 0x30
#define LCD_ONE_LINE_4BIT_MODE 0x20

// commands
#define LCD_CLEARDISPLAY   0x01
#define LCD_RETURNHOME     0x02
#define LCD_ENTRYMODESET   0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT    0x10
#define LCD_FUNCTIONSET    0x20
#define LCD_SETCGRAMADDR   0x40
#define LCD_SETDDRAMADDR   0x80

#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON  0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON   0x02
#define LCD_CURSOROFF  0x00
#define LCD_BLINKON    0x01
#define LCD_BLINKOFF   0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE  0x00
#define LCD_MOVERIGHT   0x04
#define LCD_MOVELEFT    0x00


I2CStatus initLCD(I2C_TypeDef *I2Cx);
void commandLCD(uint8_t command);

void clearLCD();

void turnOnLCD();
void turnOffLCD();

void returnHomeLCD();
void moveCursorAtSecondLineBeginningLCD();

void enableBlinkingCursorLCD();
void disableBlinkingCursorLCD();
void enableCursorLCD();
void disableCursorLCD();
void moveCursorLeftLCD();
void moveCursorRightLCD();
void setCursorIncrementFromLeftToRightLCD();
void setCursorIncrementFromRightToLeftLCD();

void moveDisplayLeftLCD();
void moveDisplayRightLCD();

void enableAutoScrollLCD();
void disableAutoScrollLCD();

void printLCD(char *string);
void printfLCD(char *format, ...);
void cleanPrintLCD(const char *string);
void cleanPrintfLCD(char *format, ...);
void printLCDChar(unsigned char charData);

void goToXYLCD(uint8_t row, uint8_t pos);
void printAtPositionLCD(uint8_t row, uint8_t pos, char *str);
void printfAtPositionLCD(uint8_t row, uint8_t pos, char *format, ...);
void cleanPrintAtPositionLCD(uint8_t row, uint8_t col, const char *string);
void cleanPrintfAtPositionLCD(uint8_t row, uint8_t col, char *format, ...);

void createCustomCharLCD(uint8_t location, const uint8_t *charmap);
void printCustomCharLCD(uint8_t location);

void initProgressBar(uint8_t len, uint8_t row, uint8_t col);
void incrementProgressBar();
void decrementProgressBar();