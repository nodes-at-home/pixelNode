/*
 * MaxMatrix
 * Version 1.0 Feb 2013 by Oscar Kin-Chung Au
 * Version 2.0 Aug 2016 by Andreas Jungierek
 */

#ifndef _MaxMatrix_H_
#define _MaxMatrix_H_

#include "Arduino.h"

#define max7219_reg_noop        0x00
#define max7219_reg_digit0      0x01
#define max7219_reg_digit1      0x02
#define max7219_reg_digit2      0x03
#define max7219_reg_digit3      0x04
#define max7219_reg_digit4      0x05
#define max7219_reg_digit5      0x06
#define max7219_reg_digit6      0x07
#define max7219_reg_digit7      0x08
#define max7219_reg_decodeMode  0x09
#define max7219_reg_intensity   0x0a
#define max7219_reg_scanLimit   0x0b
#define max7219_reg_shutdown    0x0c
#define max7219_reg_displayTest 0x0f

const int NUM_DISPLAY_COLS = 8 * 8;
const int NUM_BUF_COLS = 3 * NUM_DISPLAY_COLS;

class MaxMatrix
{
  private:
    byte data;
    byte load;
    byte clock;
    byte num;
    
  public:

    byte pixelBuffer [NUM_BUF_COLS];

    MaxMatrix ( byte data, byte load, byte clock, byte num );
    
    void shiftOutCol ( byte col, byte value );
    void shiftOutMatrix ( int startCol, int verticalShift );
    void shiftOutMatrix ( int startCol );

    void printSprite ( int col, const byte* sprite );
    int printEmptyCol ( int col );
    int printChar ( char c, int atColumn );
    
    void init();
    void clear();
    void setCommand(byte command, byte value);
    void setIntensity(byte intensity);
    
};

#endif
