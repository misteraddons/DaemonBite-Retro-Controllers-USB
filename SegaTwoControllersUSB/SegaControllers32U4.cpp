//
// SegaControllers32U4.cpp
//
// Authors:
//       Jon Thysell <thysell@gmail.com>
//       Mikael Norrgård <mick@daemonbite.com>
//
// (Based on the code by Jon Thysell, but the interfacing is almost completely
//  rewritten by Mikael Norrgård)
//
// Copyright (c) 2017 Jon Thysell <http://jonthysell.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "Arduino.h"
#include "SegaControllers32U4.h"

SegaControllers32U4::SegaControllers32U4(void)
{
/*
https://docs.arduino.cc/hacking/hardware/PinMapping32u4
A0     = PF7 [P2-3 / HDMI2P3 / DE92P3 / GND]
A1     = PF6 [P2-2 / HDMI2P2 / DE92P2 / P1DOWN
A2     = PF5 [P2-1 / HDMI2P1 / DE92P2 / P2UP]
A3     = PF4 [J2_ID]
A4     = PF1 ------
A5     = PF0 ------
D0/RX  = PD2 [P1-9]
D1/TX  = PD3 [P1-8]
D2/SDA = PD1 [P1-7 / HDMI1P7 / DE91P9 / START|C]
D3/SCL = PD0 [P1-6 / HDMI1P6 / DE91P7 / SELECT]
D4     = PD4 [J1_ID]
D5     = PC6 [P1-5 / HDMI1P5 / DE91P6 / A]
D6     = PD7 [P1-4 / HDMI1P4 / DE91P4 / GND]
D7     = PE6 [P1-3 / HDMI1P3 / DE91P3 / GND]
D8     = PB4 [P1-2 / HDMI1P2 / DE91P2 / P1DOWN]
D9     = PB5 [P1-1 / HDMI1P1 / DE91P1 / P1UP]
D10    = PB6 [P2-7 / HDMI2P7 / DE92P9 / START|C]
D11    = PB7 ------
D12    = PD6 ------
D13    = PC7 ------
D14    = PB3 [P2-5 / HDMI2P5 / DE92P6 / A]
D15    = PB1 [P2-4 / HDMI2P4 / DE92P4 / GND]
D16    = PB2 [P2-6 / HDMI2P6 / DE92P7 / SELECT]

Controller DB9 pins (looking face-on to the end of the plug):

 5 4 3 2 1
  9 8 7 6

Connect pin 5 to +5V and pin 8 to GND
Connect the remaining pins to digital I/O pins (see below)
P1 DB9    Arduino Pro Micro
--------------------------------------
 1        A0  PF7
 2        A1  PF6
 3        A2  PF5
 4        A3  PF4
 6        14  PB3
 7         7  PE6
 9        15  PB1

P2 DB9    Arduino Pro Micro
--------------------------------------
 1     D0/TX  PD3
 2     D1/RX  PD2
 3        D2  PD1
 4        D3  PD0
 6        D4  PD4
 7        D5  PC6
 9        D6  PD7
*/

  // Setup input pins (A0,A1,A2,A3,14,15 or PF7,PF6,PF5,PF4,PB3,PB1)
  //DDRF  &= ~B11110000; // input (PF7, PF6, PF5, PF4, !PF3, !PF2, !PF1, !PF0)
  //PORTF |=  B11110000; // high to enable internal pull-up (PF7, PF6, PF5, PF4, !PF3, !PF2, !PF1, !PF0)
  //DDRB  &= ~B00001010; // input (!PB7, !PB6, !PB5, !PB4, PB3, !PB2, PB1, !PB0)
  //PORTB |=  B00001010; // high to enable internal pull-up (PB7, PB6, PB5, PB4, !PB3, !PB2, !PB1, !PB0)
  // Setup input pins (TXO,RXI,2,3,4,6 or PD3,PD2,PD1,PD0,PD4,PD7)
  //DDRD  &= ~B10011111; // input (PD7, !PD6, !PD5, PD4, PD3, PD2, PD1, PD0)
  //PORTD |=  B10011111; // high to enable internal pull-up (PF7, PF6, PF5, PF4, !PF3, !PF2, !PF1, !PF0)
  //DDRC  |=  B01000000; // Select pins as output (!PC7, PC6, !PC5, !PC4, !PC3, !PC2, !PC1, !PC0)
  //DDRE  |=  B01000000; // Select pins as output (!PE7, PE6, !PE5, !PE4, !PE3, !PE2, !PE1, !PE0)
  //PORTC |=  B01000000; // Select pins high (!PC7, PC6, !PC5, !PC4, !PC3, !PC2, !PC1, !PC0)
  //PORTE |=  B01000000; // Select pins  high (!PE7, PE6, !PE5, !PE4, !PE3, !PE2, !PE1, !PE0)
  // Setup output pins
  //PORTC |=  B01000000; // Select pins high (!PC7, PC6, !PC5, !PC4, !PC3, !PC2, !PC1, !PC0)
  //PORTE |=  B01000000; // Select pins  high (!PE7, PE6, !PE5, !PE4, !PE3, !PE2, !PE1, !PE0)
  
  // Setup input pins
  DDRB  &= ~B01111010;
  PORTB |=  B01111010;
  DDRC  &= ~B01000000;
  PORTC |=  B01000000;
  DDRD  &= ~B10000010;
  PORTD |=  B10000010;
  DDRE  &= ~B01000000;
  PORTE |=  B01000000;
  DDRF  &= ~B11100000;
  PORTF |=  B11100000;
  
  // Setup output pins
  PORTB |=  B00001000;
  PORTD |=  B00000001;
  
  _pinSelect = true;
  for(byte i=0; i<=1; i++)
  {
    currentState[i] = 0;
    _connected[i] = 0;
    _sixButtonMode[i] = false;
    _ignoreCycles[i] = 0;
  }
}

void SegaControllers32U4::readState()
{
  // Set the select pins low/high
  _pinSelect = !_pinSelect;
  if(!_pinSelect) {
    PORTB &= ~B00001000;
    PORTD &= ~B00000001;
  } else {
    PORTB |=  B00001000;
    PORTD |=  B00000001;
  }

  // Short delay to stabilise outputs in controller
  delayMicroseconds(SC_CYCLE_DELAY);

  // Read all input registers
//  _inputReg1 = PINB;
//  _inputReg2 = PINC;
//  _inputReg3 = PIND;
//  _inputReg4 = PINE;
//  _inputReg5 = PINF;

  readPort1();
  readPort2();
}

// "Normal" Six button controller reading routine, done a bit differently in this project
// Cycle  TH out  TR in  TL in  D3 in  D2 in  D1 in  D0 in
// 0      LO      Start  A      0      0      Down   Up      
// 1      HI      C      B      Right  Left   Down   Up
// 2      LO      Start  A      0      0      Down   Up      (Check connected and read Start and A in this cycle)
// 3      HI      C      B      Right  Left   Down   Up      (Read B, C and directions in this cycle)
// 4      LO      Start  A      0      0      0      0       (Check for six button controller in this cycle)
// 5      HI      C      B      Mode   X      Y      Z       (Read X,Y,Z and Mode in this cycle)    
// 6      LO      ---    ---    ---    ---    ---    Home    (Home only for 8bitdo wireless gamepads)      
// 7      HI      ---    ---    ---    ---    ---    ---    

void SegaControllers32U4::readPort1()
{
  if(_ignoreCycles[0] <= 0)
  {
    if(_pinSelect) // Select pin is HIGH
    {
      if(_connected[0])
      {
        // Check if six button mode is active
        if(_sixButtonMode[0])
        {
          Serial.println("J1 6 button mode");
          // Read input pins for X, Y, Z, Mode
          (bitRead(PINB, 5) == LOW) ? currentState[0] |= SC_BTN_Z : currentState[0] &= ~SC_BTN_Z;
          (bitRead(PINB, 4) == LOW) ? currentState[0] |= SC_BTN_Y : currentState[0] &= ~SC_BTN_Y;
          (bitRead(PINE, 6) == LOW) ? currentState[0] |= SC_BTN_X : currentState[0] &= ~SC_BTN_X;
          (bitRead(PIND, 7) == LOW) ? currentState[0] |= SC_BTN_MODE : currentState[0] &= ~SC_BTN_MODE;
          _sixButtonMode[0] = false;
          _ignoreCycles[0] = 2; // Ignore the two next cycles (cycles 6 and 7 in table above)
        }
        else
        {
          Serial.println("J1 3 button mode");
          // Read input pins for Up, Down, Left, Right, B, C
          (bitRead(PINB, 5) == LOW) ? currentState[0] |= SC_BTN_UP : currentState[0] &= ~SC_BTN_UP;
          (bitRead(PINB, 4) == LOW) ? currentState[0] |= SC_BTN_DOWN : currentState[0] &= ~SC_BTN_DOWN;
          (bitRead(PINE, 6) == LOW) ? currentState[0] |= SC_BTN_LEFT : currentState[0] &= ~SC_BTN_LEFT;
          (bitRead(PIND, 7) == LOW) ? currentState[0] |= SC_BTN_RIGHT : currentState[0] &= ~SC_BTN_RIGHT;
          (bitRead(PINC, 6) == LOW) ? currentState[0] |= SC_BTN_B : currentState[0] &= ~SC_BTN_B;
          (bitRead(PIND, 1) == LOW) ? currentState[0] |= SC_BTN_C : currentState[0] &= ~SC_BTN_C;
        }
      }
      else // No Mega Drive controller is connected, use SMS/Atari mode
      {
        // Clear current state
        currentState[0] = 0;
        Serial.println("J1 SMS/Atari mode");
        // Read input pins for Up, Down, Left, Right, Fire1, Fire2
        if (bitRead(PINB, 5) == LOW) { currentState[0] |= SC_BTN_UP; }
        if (bitRead(PINB, 4) == LOW) { currentState[0] |= SC_BTN_DOWN; }
        if (bitRead(PINE, 6) == LOW) { currentState[0] |= SC_BTN_LEFT; }
        if (bitRead(PIND, 7) == LOW) { currentState[0] |= SC_BTN_RIGHT; }
        if (bitRead(PINC, 6) == LOW) { currentState[0] |= SC_BTN_A; }
        if (bitRead(PIND, 1) == LOW) { currentState[0] |= SC_BTN_B; }
      }
    }
    else // Select pin is LOW
    {
      // Check if a controller is connected
      _connected[0] = (bitRead(PINE, 6) == LOW && bitRead(PIND, 7) == LOW);
      
      // Check for six button mode
      _sixButtonMode[0] = (bitRead(PINB, 5) == LOW && bitRead(PINB, 4) == LOW);
      
      // Read input pins for A and Start 
      if(_connected[0])
      {
        Serial.println("J1 connected");
        if(!_sixButtonMode[0])
        {
          (bitRead(PINC, 6) == LOW) ? currentState[0] |= SC_BTN_A : currentState[0] &= ~SC_BTN_A;
          (bitRead(PIND, 1) == LOW) ? currentState[0] |= SC_BTN_START : currentState[0] &= ~SC_BTN_START; 
        }
      }
    }
  }
  else
  {
    if(_ignoreCycles[0]-- == 2) // Decrease the ignore cycles counter and read 8bitdo home in first "ignored" cycle, this cycle is unused on normal 6-button controllers
    {
      (bitRead(PINB, 5) == LOW) ? currentState[0] |= SC_BTN_HOME : currentState[0] &= ~SC_BTN_HOME;
    }
  }
}

void SegaControllers32U4::readPort2()
{
  if(_ignoreCycles[1] <= 0)
  {
    if(_pinSelect) // Select pin is HIGH
    {
      if(_connected[1])
      {
        // Check if six button mode is active
        if(_sixButtonMode[1])
        {
          // Read input pins for X, Y, Z, Mode
          (bitRead(PINF, 5) == LOW) ? currentState[1] |= SC_BTN_Z : currentState[1] &= ~SC_BTN_Z;
          (bitRead(PINF, 6) == LOW) ? currentState[1] |= SC_BTN_Y : currentState[1] &= ~SC_BTN_Y;
          (bitRead(PINF, 7) == LOW) ? currentState[1] |= SC_BTN_X : currentState[1] &= ~SC_BTN_X;
          (bitRead(PINB, 1) == LOW) ? currentState[1] |= SC_BTN_MODE : currentState[1] &= ~SC_BTN_MODE;
          _sixButtonMode[1] = false;
          _ignoreCycles[1] = 2; // Ignore the two next cycles (cycles 6 and 7 in table above)
        }
        else
        {
          // Read input pins for Up, Down, Left, Right, B, C
          (bitRead(PINF, 5) == LOW) ? currentState[1] |= SC_BTN_UP : currentState[1] &= ~SC_BTN_UP;
          (bitRead(PINF, 6) == LOW) ? currentState[1] |= SC_BTN_DOWN : currentState[1] &= ~SC_BTN_DOWN;
          (bitRead(PINF, 7) == LOW) ? currentState[1] |= SC_BTN_LEFT : currentState[1] &= ~SC_BTN_LEFT;
          (bitRead(PINB, 1) == LOW) ? currentState[1] |= SC_BTN_RIGHT : currentState[1] &= ~SC_BTN_RIGHT;
          (bitRead(PINB, 3) == LOW) ? currentState[1] |= SC_BTN_B : currentState[1] &= ~SC_BTN_B;
          (bitRead(PINB, 6) == LOW) ? currentState[1] |= SC_BTN_C : currentState[1] &= ~SC_BTN_C;
        }
      }
      else // No Mega Drive controller is connected, use SMS/Atari mode
      {
        // Clear current state
        currentState[1] = 0;
        
        // Read input pins for Up, Down, Left, Right, Fire1, Fire2
        if (bitRead(PINF, 5) == LOW) { currentState[1] |= SC_BTN_UP; }
        if (bitRead(PINF, 6) == LOW) { currentState[1] |= SC_BTN_DOWN; }
        if (bitRead(PINF, 7) == LOW) { currentState[1] |= SC_BTN_LEFT; }
        if (bitRead(PINB, 1) == LOW) { currentState[1] |= SC_BTN_RIGHT; }
        if (bitRead(PINB, 3) == LOW) { currentState[1] |= SC_BTN_A; }
        if (bitRead(PINB, 6) == LOW) { currentState[1] |= SC_BTN_B; }
      }
    }
    else // Select pin is LOW
    {
      // Check if a controller is connected
      _connected[1] = (bitRead(PINF, 7) == LOW && bitRead(PINB, 1) == LOW);
      
      // Check for six button mode
      _sixButtonMode[1] = (bitRead(PINF, 5) == LOW && bitRead(PINF, 6) == LOW);
      
      // Read input pins for A and Start 
      if(_connected[1])
      {
        if(!_sixButtonMode[1])
        {
          (bitRead(PINB, 3) == LOW) ? currentState[1] |= SC_BTN_A : currentState[1] &= ~SC_BTN_A;
          (bitRead(PINB, 6) == LOW) ? currentState[1] |= SC_BTN_START : currentState[1] &= ~SC_BTN_START; 
        }
      }
    }
  }
  else
  {
    if(_ignoreCycles[1]-- == 2) // Decrease the ignore cycles counter and read 8bitdo home in first "ignored" cycle, this cycle is unused on normal 6-button controllers
    {
      (bitRead(PINF, 5) == LOW) ? currentState[1] |= SC_BTN_HOME : currentState[1] &= ~SC_BTN_HOME;
    }
  }
}
