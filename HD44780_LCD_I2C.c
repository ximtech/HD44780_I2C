#include "HD44780_LCD_I2C.h"

// PCF8574 output pin mapping (P0 - P7)
#define LCD_RS 0
#define LCD_RW 1
#define LCD_EN 2
#define LCD_BL 3

static I2C_Polling i2c = {0};

static uint8_t displayControlParameters;
static char formatBuffer[HD44780_LCD_COL_COUNT + 1];

static const uint8_t loadBarStartElement[8] = {0b10000, 0b11000, 0b11100, 0b11110, 0b11110, 0b11100, 0b11000, 0b10000};
static const uint8_t loadBarProgressElement[8] = {0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
static const uint8_t loadBarEndElement[8] = {0b00001, 0b00011, 0b00111, 0b01111, 0b01111, 0b00111, 0b00011, 0b00001};

uint8_t barRowLocation = 0;
uint8_t barColLocation = 0;
uint8_t barTotalLength = 0;
uint8_t barProgress = 0;


I2CStatus initLCD(I2C_TypeDef *I2Cx) {
    i2c = initI2C(I2Cx, I2C_ADDRESSING_MODE_7BIT, HD44780_LCD_I2C_TIMEOUT_MS);

    I2CStatus status = isDeviceReady(&i2c, HD44780_LCD_I2C_DEVICE_ADDRESS);
    if (status != I2C_OK) {
        return status;
    }
    delay_ms(20);		// LCD Power ON delay always >15ms

    commandLCD(0x33);
    commandLCD(0x32);    // Send for 4 bit initialization of LCD
    commandLCD(LCD_TWO_LINES_5X8_MATRIX_4BIT_MODE);

    commandLCD(LCD_DISPLAY_ON_CURSOR_OFF);
    commandLCD(LCD_SHIFT_CURSOR_RIGHT);        // Auto Increment cursor from left to right
    clearLCD();
    return status;
}

void commandLCD(uint8_t command) {
    if (startAsMasterI2C(&i2c, HD44780_LCD_I2C_DEVICE_ADDRESS, I2C_WRITE_TO_SLAVE) == I2C_OK) {
        // send higher bits to LCD
        uint8_t LCD = 0;
        LCD |= (command & 0xF0);  // get upper 4 bits
        LCD |= (1 << LCD_EN);     // enable pulse
        LCD |= (1 << LCD_BL);     // put unused pin to high

        LCD &= ~(1 << LCD_RS);    // explicitly set RS and RW to low for command
        LCD &= ~(1 << LCD_RW);
        transmitByteAsMasterI2C(&i2c, LCD);
        delay_us(1);

        LCD &= ~(1 << LCD_EN);
        transmitByteAsMasterI2C(&i2c, LCD);
        delay_us(200);

        // send lower bits to LCD
        LCD = 0;
        LCD |= (command << 4);  // get lower 4 bits
        LCD |= (1 << LCD_EN);   // enable pulse
        LCD |= (1 << LCD_BL);   // put unused pin to high

        LCD &= ~(1 << LCD_RS);    // explicitly set RS and RW to low for command
        LCD &= ~(1 << LCD_RW);
        transmitByteAsMasterI2C(&i2c, LCD);
        delay_us(1);

        LCD &= ~(1 << LCD_EN);
        transmitByteAsMasterI2C(&i2c, LCD);
        delay_ms(2);

        stopAsMasterI2C(&i2c);
    }
}

void printLCDChar(unsigned char charData) {
    if (startAsMasterI2C(&i2c, HD44780_LCD_I2C_DEVICE_ADDRESS, I2C_WRITE_TO_SLAVE) == I2C_OK) {
        // send higher bits to LCD
        uint8_t LCD = 0;
        LCD |= (charData & 0xF0);   // get upper 4 bits
        LCD |= (1 << LCD_EN);        // enable pulse
        LCD |= (1 << LCD_RS);        // RS=1, data reg.
        LCD |= (1 << LCD_BL);        // put unused pin to high

        LCD &= ~(1 << LCD_RW);    // set RW low for data send
        transmitByteAsMasterI2C(&i2c, LCD);
        delay_us(1);

        LCD &= ~(1 << LCD_EN);
        transmitByteAsMasterI2C(&i2c, LCD);
        delay_us(200);

        // send lower bits to LCD
        LCD = 0;
        LCD |= (charData << 4);
        LCD |= (1 << LCD_EN);
        LCD |= (1 << LCD_RS);        // RS=1, data reg.
        LCD |= (1 << LCD_BL);

        LCD &= ~(1 << LCD_RW);
        transmitByteAsMasterI2C(&i2c, LCD);
        delay_us(1);

        LCD &= ~(1 << LCD_EN);
        transmitByteAsMasterI2C(&i2c, LCD);
        delay_ms(2);

        stopAsMasterI2C(&i2c);
    }
}

void clearLCD() {
    commandLCD(LCD_CLEAR_DISPLAY);
    delay_ms(2);        // Clear display command delay > 1.63 ms
    returnHomeLCD();
}

void turnOnLCD() {
    displayControlParameters |= LCD_DISPLAYON;
    commandLCD(LCD_DISPLAYCONTROL | displayControlParameters);
}

void turnOffLCD() {
    displayControlParameters &= ~LCD_DISPLAYON;
    commandLCD(LCD_DISPLAYCONTROL | displayControlParameters);
}

void returnHomeLCD() {
    commandLCD(LCD_MOVE_CURSOR_AT_FIRST_LINE_BEGINNING);
}

void enableBlinkingCursorLCD() {
    displayControlParameters |= LCD_DISPLAY_ON_CURSOR_BLINKING;
    commandLCD(LCD_DISPLAYCONTROL | displayControlParameters);
}

void disableBlinkingCursorLCD() {
    commandLCD(LCD_DISPLAY_ON_CURSOR_OFF);
}

void enableCursorLCD() {
    displayControlParameters |= LCD_CURSORON;
    commandLCD(LCD_DISPLAYCONTROL | displayControlParameters);
}

void disableCursorLCD() {
    displayControlParameters &= ~LCD_CURSORON;
    commandLCD(LCD_DISPLAYCONTROL | displayControlParameters);
}

void moveCursorLeftLCD() {
    commandLCD(LCD_SHIFT_POSITION_LEFT);
}

void moveCursorRightLCD() {
    commandLCD(LCD_SHIFT_POSITION_LEFT);
}

void moveCursorAtSecondLineBeginningLCD() {
    commandLCD(LCD_MOVE_CURSOR_AT_SECOND_LINE_BEGINNING);
}

void moveDisplayLeftLCD() {
    commandLCD(LCD_SHIFT_DISPLAY_LEFT);
}

void moveDisplayRightLCD() {
    commandLCD(LCD_SHIFT_DISPLAY_RIGHT);
}

void setCursorIncrementFromLeftToRightLCD() {
    displayControlParameters |= LCD_ENTRYLEFT;
    commandLCD(LCD_ENTRYMODESET | displayControlParameters);
}

void setCursorIncrementFromRightToLeftLCD() {
    displayControlParameters &= ~LCD_ENTRYLEFT;
    commandLCD(LCD_ENTRYMODESET | displayControlParameters);
}

void enableAutoScrollLCD() {
    displayControlParameters |= LCD_ENTRYSHIFTINCREMENT;
    commandLCD(LCD_ENTRYMODESET | displayControlParameters);
}

void disableAutoScrollLCD() {
    displayControlParameters &= ~LCD_ENTRYSHIFTINCREMENT;
    commandLCD(LCD_ENTRYMODESET | displayControlParameters);
}

void goToXYLCD(uint8_t row, uint8_t pos) {
    static const uint8_t rowOffsets[] = {0x00, 0x40, 0x14, 0x54};

    if (row < HD44780_LCD_ROW_COUNT && pos < HD44780_LCD_COL_COUNT) {
        commandLCD(LCD_SETDDRAMADDR | (pos + rowOffsets[row]));
    }
}

void printLCD(char *string) {
    for (uint8_t i = 0; string[i] != 0; i++) {
        printLCDChar(string[i]);
    }
}

void printAtPositionLCD(uint8_t row, uint8_t pos, char *str) {            // Send string to LCD with xy position
    goToXYLCD(row, pos);
    printLCD(str);
}

void printfAtPositionLCD(uint8_t row, uint8_t pos, char *format, ...) {
    va_list args;

    va_start(args, format);
    vsnprintf(formatBuffer, HD44780_LCD_COL_COUNT + 1, format, args);
    va_end(args);

    goToXYLCD(row, pos);
    printLCD(formatBuffer);
}

void cleanPrintAtPositionLCD(uint8_t row, uint8_t col, const char *string) {
    char buffer[HD44780_LCD_COL_COUNT] = {[0 ... HD44780_LCD_COL_COUNT - 1] = ' '};
    buffer[HD44780_LCD_COL_COUNT - 1] = '\0';
    for (uint8_t i = col, j = 0; i < HD44780_LCD_COL_COUNT && string[j] != '\0'; i++, j++) {
        buffer[i] = string[j];
    }
    goToXYLCD(row, 0);
    printLCD(buffer);
}

void cleanPrintfAtPositionLCD(uint8_t row, uint8_t col, char *format, ...) {
    va_list args;

    va_start(args, format);
    vsnprintf(formatBuffer, HD44780_LCD_COL_COUNT + 1, format, args);
    va_end(args);

    cleanPrintAtPositionLCD(row, col, formatBuffer);
}

void createCustomCharLCD(uint8_t location, const uint8_t *charmap) {    // location from 0 to 7
    if (location < 8) {
        commandLCD(LCD_SETCGRAMADDR + (location * 8));// Command 0x40 and onwards forces the device to point CGRAM address
        for (uint8_t i = 0; i < 8; i++) {
            printLCDChar(charmap[i]);
        }
        returnHomeLCD();
    }
}

void printCustomCharLCD(uint8_t location) {    // location from 0 to 7
    if (location < 8) {
        printLCDChar(location);
    }
}

void printfLCD(char *format, ...) {
    va_list args;

    va_start(args, format);
    vsnprintf(formatBuffer, HD44780_LCD_COL_COUNT + 1, format, args);
    va_end(args);

    printLCD(formatBuffer);
}

void cleanPrintLCD(const char *string) {
    char buffer[HD44780_LCD_COL_COUNT] = {[0 ... HD44780_LCD_COL_COUNT - 1] = ' '};
    buffer[HD44780_LCD_COL_COUNT - 1] = '\0';
    for (uint8_t i = 0; i < HD44780_LCD_COL_COUNT && string[i] != '\0'; i++) {
        buffer[i] = string[i];
    }
    printLCD(buffer);
}

void cleanPrintfLCD(char *format, ...) {
    va_list args;

    va_start(args, format);
    vsnprintf(formatBuffer, HD44780_LCD_COL_COUNT + 1, format, args);
    va_end(args);

    cleanPrintLCD(formatBuffer);
}

void initProgressBar(uint8_t len, uint8_t row, uint8_t col) {
    bool isBarLengthValid = (len <= HD44780_LCD_COL_COUNT - 1) && ((len - 2) >= 0);
    bool isRowValid = row < HD44780_LCD_ROW_COUNT;

    if (isBarLengthValid && isRowValid) {
        createCustomCharLCD(0, loadBarStartElement);
        createCustomCharLCD(1, loadBarProgressElement);
        createCustomCharLCD(2, loadBarEndElement);

        barRowLocation = row;
        barColLocation = col + 1;
        barTotalLength = len - 1;

        goToXYLCD(row, col);
        printCustomCharLCD(0);

        goToXYLCD(row, (col + len));
        printCustomCharLCD(2);
    }
}

void incrementProgressBar() {
    if (barTotalLength > 0 && barProgress < barTotalLength) {
        goToXYLCD(barRowLocation, barColLocation++);
        printCustomCharLCD(1);
        barProgress++;
    }
}

void decrementProgressBar() {
    if (barTotalLength > 0 && barProgress > 0) {
        goToXYLCD(barRowLocation, barProgress--);
        printLCD(" ");
        barColLocation--;
    }
}