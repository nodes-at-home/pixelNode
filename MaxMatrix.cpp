/*
 * MaxMatrix
 * Version 1.0 Feb 2013 by Oscar Kin-Chung Au
 * Version 2.0 Aug 2016 by Andreas Jungierek
 */


#include "Arduino.h"
#include "MaxMatrix.h"
#include "Sprites.h"

MaxMatrix::MaxMatrix ( byte _data, byte _load, byte _clock, byte _num ) {

    data = _data;
    load = _load;
    clock = _clock;
    num = _num;
    for ( int i = 0; i < NUM_BUF_COLS; i++) pixelBuffer [i] = 0;

}

void MaxMatrix::init () {
	
	init ( 0x00 );

}

void MaxMatrix::init ( byte intensity ) {

  pinMode ( data, OUTPUT );
  pinMode ( clock, OUTPUT );
  pinMode ( load, OUTPUT );

  digitalWrite ( clock, HIGH );

  setCommand ( max7219_reg_scanLimit, 0x07 );
  setCommand ( max7219_reg_decodeMode, 0x00 );  // using an led matrix (not digits)
  setCommand ( max7219_reg_shutdown, 0x01 );    // not in shutdown mode
  setCommand ( max7219_reg_scanLimit, 0x07 );    // all 8 digits
  setCommand ( max7219_reg_displayTest, 0x00 ); // no display test
  setCommand ( max7219_reg_intensity, intensity );

  // empty registers, turn all LEDs off
  clear ();

}

void MaxMatrix::setIntensity ( byte intensity ) {

  setCommand ( max7219_reg_intensity, intensity );

}

void MaxMatrix::clear () {

  for ( int i = 0; i < NUM_BUF_COLS; i++ ) pixelBuffer [i] = 0;
  shiftOutMatrix ( NUM_DISPLAY_COLS );

}

void MaxMatrix::shiftOutCol ( byte col, byte value ) {

  shiftOut( data, clock, MSBFIRST, col );
  shiftOut( data, clock, MSBFIRST, value );

}

void MaxMatrix::shiftOutMatrix ( int startCol ) {

  shiftOutMatrix ( startCol, 0 );

}

void MaxMatrix::shiftOutMatrix ( int startCol, int verticalShift ) {

  verticalShift = constrain ( verticalShift, -8, +8 );

  for ( int col = 0; col < 8; col++ ) {

    digitalWrite ( load, LOW ); // state befor is HIGH

    for ( int i = 0; i < num; i++ ) {
      int c = startCol + 8*i + col;
      if ( verticalShift == 0 ) {
        shiftOutCol ( col+1, pixelBuffer [c] );
      }
      else {
        if ( verticalShift > 0 ) {
          shiftOutCol ( col+1, pixelBuffer [c] >> verticalShift );
        }
        else {
          shiftOutCol ( col+1, pixelBuffer [c] << -verticalShift );
        }
      }
    }

    digitalWrite ( load, LOW );
    digitalWrite ( load, HIGH );

  }

}

void MaxMatrix::printSprite ( int col, const byte* sprite ) {

  int w = sprite [0];

  for ( int i = 0; i < w; i++ ) {
    int c = col + i;
    if ( c >= 0 && c < NUM_BUF_COLS )
      pixelBuffer [c] = sprite [i + 2];
  }

}

int MaxMatrix::printEmptyCol ( int col ) {

    if ( col >= 0 && col < NUM_BUF_COLS ) pixelBuffer [col] = 0;

    return 1;

}

int MaxMatrix::printChar ( char c, int atColumn ) {

    int insertCol = atColumn;

    if ( atColumn < NUM_BUF_COLS ) {
        if ( c >= 32 ) {
            c -= 32;
            if ( c < 103 ) {
                const byte* sprite = SPRITE + 7*c;
                printSprite ( insertCol, sprite );
                insertCol += sprite [0];
                printEmptyCol ( insertCol++ );
            }
        }
    }

    return insertCol - atColumn;

}

void MaxMatrix::setCommand ( byte command, byte value ) {

  digitalWrite ( load, LOW );
  for ( int i = 0; i < num; i++ ) {
    shiftOut(data, clock, MSBFIRST, command);
    shiftOut(data, clock, MSBFIRST, value);
  }
  digitalWrite(load, LOW);
  digitalWrite(load, HIGH);

}

