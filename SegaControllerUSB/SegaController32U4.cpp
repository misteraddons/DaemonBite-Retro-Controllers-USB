//
// SegaController32U4.cpp
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
#include "SegaController32U4.h"

SegaController32U4::SegaController32U4(void)
{
/*
https://docs.arduino.cc/hacking/hardware/PinMapping32u4
A0     = PF7 [P2-3]
A1     = PF6 [P2-2]
A2     = PF5 [P2-1]
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
D10    = PB6 [P2-7]
D11    = PB7 ------
D12    = PD6 ------
D13    = PC7 ------
D14    = PB3 [P2-5]
D15    = PB1 [P2-4]
D16    = PB2 [P2-6]

Controller DB9 pins (looking face-on to the end of the plug):

 5 4 3 2 1
  9 8 7 6

Connect pin 5 to +5V and pin 8 to GND
Connect the remaining pins to digital I/O pins (see below)
DB9    Arduino Pro Micro
--------------------------------------
 1     A0  PF7
 2     A1  PF6
 3     A2  PF5
 4     A3  PF4
 6     14  PB3
 7      7  PE6
 9     15  PB1
*/

    // Setup select pin as output high (7, PE6)
    DDR_SELECT  |= MASK_SELECT; // output
    PORT_SELECT |= MASK_SELECT; // high

    // Setup input pins (A0,A1,A2,A3,14,15 or PF7,PF6,PF5,PF4,PB3,PB1)
    DDRF  &= ~B11110000; // input
    PORTF |=  B11110000; // high to enable internal pull-up
    DDRB  &= ~B00001010; // input
    PORTB |=  B00001010; // high to enable internal pull-up
    
    _inputReg1 = 0;
    _inputReg2 = 0;
    _currentState = 0;
    _connected = 0;
    _sixButtonMode = false;
    _ignoreCycles = 0;
    _pinSelect = true;
}

word SegaController32U4::getStateMD()
{
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

  // Set the select pin low/high
  _pinSelect = !_pinSelect;
  (!_pinSelect) ? PORT_SELECT &= ~MASK_SELECT : PORT_SELECT |= MASK_SELECT; // Set LOW on even cycle, HIGH on uneven cycle

  // Short delay to stabilise outputs in controller
  delayMicroseconds(SC_CYCLE_DELAY);

  // Read input register(s)
  _inputReg1 = PINF;
  _inputReg2 = PINB;

  if(_ignoreCycles <= 0)
  {
    if(_pinSelect) // Select pin is HIGH
    {
      if(_connected)
      {
        // Check if six button mode is active
        if(_sixButtonMode)
        {
          // Read input pins for X, Y, Z, Mode
          (bitRead(_inputReg1, DB9_PIN1_BIT) == LOW) ? _currentState |= SC_BTN_Z : _currentState &= ~SC_BTN_Z;
          (bitRead(_inputReg1, DB9_PIN2_BIT) == LOW) ? _currentState |= SC_BTN_Y : _currentState &= ~SC_BTN_Y;
          (bitRead(_inputReg1, DB9_PIN3_BIT) == LOW) ? _currentState |= SC_BTN_X : _currentState &= ~SC_BTN_X;
          (bitRead(_inputReg1, DB9_PIN4_BIT) == LOW) ? _currentState |= SC_BTN_MODE : _currentState &= ~SC_BTN_MODE;
          _sixButtonMode = false;
          _ignoreCycles = 2; // Ignore the two next cycles (cycles 6 and 7 in table above)
        }
        else
        {
          // Read input pins for Up, Down, Left, Right, B, C
          (bitRead(_inputReg1, DB9_PIN1_BIT) == LOW) ? _currentState |= SC_BTN_UP : _currentState &= ~SC_BTN_UP;
          (bitRead(_inputReg1, DB9_PIN2_BIT) == LOW) ? _currentState |= SC_BTN_DOWN : _currentState &= ~SC_BTN_DOWN;
          (bitRead(_inputReg1, DB9_PIN3_BIT) == LOW) ? _currentState |= SC_BTN_LEFT : _currentState &= ~SC_BTN_LEFT;
          (bitRead(_inputReg1, DB9_PIN4_BIT) == LOW) ? _currentState |= SC_BTN_RIGHT : _currentState &= ~SC_BTN_RIGHT;
          (bitRead(_inputReg2, DB9_PIN6_BIT) == LOW) ? _currentState |= SC_BTN_B : _currentState &= ~SC_BTN_B;
          (bitRead(_inputReg2, DB9_PIN9_BIT) == LOW) ? _currentState |= SC_BTN_C : _currentState &= ~SC_BTN_C;
        }
      }
      else // No Mega Drive controller is connected, use SMS/Atari mode
      {
        // Clear current state
        _currentState = 0;
        
        // Read input pins for Up, Down, Left, Right, Fire1, Fire2
        if (bitRead(_inputReg1, DB9_PIN1_BIT) == LOW) { _currentState |= SC_BTN_UP; }
        if (bitRead(_inputReg1, DB9_PIN2_BIT) == LOW) { _currentState |= SC_BTN_DOWN; }
        if (bitRead(_inputReg1, DB9_PIN3_BIT) == LOW) { _currentState |= SC_BTN_LEFT; }
        if (bitRead(_inputReg1, DB9_PIN4_BIT) == LOW) { _currentState |= SC_BTN_RIGHT; }
        if (bitRead(_inputReg2, DB9_PIN6_BIT) == LOW) { _currentState |= SC_BTN_A; }
        if (bitRead(_inputReg2, DB9_PIN9_BIT) == LOW) { _currentState |= SC_BTN_B; }
      }
    }
    else // Select pin is LOW
    {
      // Check if a controller is connected
      _connected = (bitRead(_inputReg1, DB9_PIN3_BIT) == LOW && bitRead(_inputReg1, DB9_PIN4_BIT) == LOW);
      
      // Check for six button mode
      _sixButtonMode = (bitRead(_inputReg1, DB9_PIN1_BIT) == LOW && bitRead(_inputReg1, DB9_PIN2_BIT) == LOW);
      
      // Read input pins for A and Start 
      if(_connected)
      {
        if(!_sixButtonMode)
        {
          (bitRead(_inputReg2, DB9_PIN6_BIT) == LOW) ? _currentState |= SC_BTN_A : _currentState &= ~SC_BTN_A;
          (bitRead(_inputReg2, DB9_PIN9_BIT) == LOW) ? _currentState |= SC_BTN_START : _currentState &= ~SC_BTN_START; 
        }
      }
    }
  }
  else
  {
    if(_ignoreCycles-- == 2) // Decrease the ignore cycles counter and read 8bitdo home in first "ignored" cycle, this cycle is unused on normal 6-button controllers
    {
      (bitRead(_inputReg1, DB9_PIN1_BIT) == LOW) ? _currentState |= SC_BTN_HOME : _currentState &= ~SC_BTN_HOME;
    }
  }

  return _currentState;
}
