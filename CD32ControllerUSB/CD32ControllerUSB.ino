/*  DaemonBite CD32 to USB Adapter
 *  Author: Mikael Norrgård <mick@daemonbite.com>
 *
 *  Copyright (c) 2020 Mikael Norrgård <http://daemonbite.com>
 *  
 *  GNU GENERAL PUBLIC LICENSE
 *  Version 3, 29 June 2007
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *  
 */

#include <EEPROM.h>
#include "Gamepad.h"

// ATT: 20 chars max (including NULL at the end) according to Arduino source code.
// Additionally serial number is used to differentiate arduino projects to have different button maps!
const char *gp_serial = "CD32/C= to USB";

#define BUTTON_READ_DELAY 100 // Button read delay in µs
#define MODE_CD32 0
#define MODE_3BUTTON 42

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
D2/SDA = PD1 [P1-7]
D3/SCL = PD0 [P1-6]
D4     = PD4 [J1_ID]
D5     = PC6 [P1-5]
D6     = PD7 [P1-4]
D7     = PE6 [P1-3]
D8     = PB4 [P1-2]
D9     = PB5 [P1-1]
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

Wire it up according to the following table:

DB9    Arduino Pro Micro
--------------------------------------
1     TXO PD3
2     RXI PD2
3      3  PD0
4      4  PD4
5     A0  PF7
6      6  PD7 (Important: Connect this pin via a 220Ω resistor!)
7     VCC
8     GND
9     A1  PF6
----------------
Second controller port for future reference)
1     15  PB1
2     14  PB3
3      2  PD1
4      5  PC6
5     A2  PF5
6      7  PE6 (Important: Connect this pin via a 220Ω resistor!)
7     VCC
8     GND
9     A3  PF4
*/


// Set up USB HID gamepad
Gamepad_ Gamepad;
bool usbUpdate = false; // Should gamepad data be sent to USB?

// Controller
uint8_t axes = 0;
uint8_t axesPrev = 0;
uint8_t buttons = 0;
uint8_t buttonsPrev = 0;

// Timing
uint32_t microsButtons = 0;
uint32_t millisStart = 0;

// CD32 controller detection
uint8_t detection = 0;
uint8_t mode = MODE_CD32;

void setup()
{
  // Setup switch pin (2, PD1)
  DDRD  &= ~B00000010; // input
  PORTD |=  B00000010; // high to enable internal pull-up

  // Setup controller pins
  DDRD  &= ~B10011101; // inputs
  PORTD |=  B10011101; // high to enable internal pull-up
  DDRF  &= ~B11000000; // input
  PORTF |=  B11000000; // high to enable internal pull-up

  delay(500);
  startupConfig();
}

void loop() { while(1)
{
  // Read X and Y axes
  axes = ~(PIND & B00011101);

  if(mode == MODE_CD32)
  {
    // See if enough time has passed since last button read
    if(micros() - microsButtons > BUTTON_READ_DELAY)
    {
      // Set pin 6 (clock, PD7) and pin 5 (latch, PF7) as output low
      PORTD &= ~B10000000; // low to disable internal pull-up (will become low when set as output)
      DDRD  |=  B10000000; // output
      PORTF &= ~B10000000; // low to disable internal pull-up (will become low when set as output)
      DDRF  |=  B10000000; // output
      delayMicroseconds(40);

      // Clear buttons
      buttons = 0;

      // Read buttons
      (PINF & B01000000) ? buttons &= ~B00000010 : buttons |= B00000010; // Blue (2)
      sendClock();
      (PINF & B01000000) ? buttons &= ~B00000001 : buttons |= B00000001; // Red (1)
      sendClock();
      (PINF & B01000000) ? buttons &= ~B00001000 : buttons |= B00001000; // Yellow (4)
      sendClock();
      (PINF & B01000000) ? buttons &= ~B00000100 : buttons |= B00000100; // Green (3)
      sendClock();
      (PINF & B01000000) ? buttons &= ~B00100000 : buttons |= B00100000; // RTrig (6)
      sendClock();
      (PINF & B01000000) ? buttons &= ~B00010000 : buttons |= B00010000; // LTrig (5)
      sendClock();
      (PINF & B01000000) ? buttons &= ~B01000000 : buttons |= B01000000; // Play (7)
      sendClock();
      (PINF & B01000000) ? detection |= B00000001 : detection &= ~B00000001; // First detection bit (should be 1)
      sendClock();
      (PINF & B01000000) ? detection |= B00000010 : detection &= ~B00000010; // Second detection bit (should be 0)

      // Set pin 5 (latch, PF7) and pin 6 (clock, PD7) as input with pull-ups
      DDRF  &= ~B10000000; // input
      PORTF |=  B10000000; // high to enable internal pull-up
      DDRD  &= ~B10000000; // input
      PORTD |=  B10000000; // high to enable internal pull-up 
      delayMicroseconds(40);

      // Was a CD32 gamepad detected? If not, read button 1 and 2 "normally".
      if(detection != B0000001)
        buttons = ~( ((PIND & B10000000) >> 7) | ((PINF & B01000000) >> 5) | B11111100 );
      
      microsButtons = micros();
    }
  }
  else
  {
    buttons = ~( ((PIND & B10000000) >> 7) | ((PINF & B11000000) >> 5) | B11111000 );
  }
  
  // Has any buttons changed state?
  if (buttons != buttonsPrev)
  {
    Gamepad._GamepadReport.buttons = buttons;
    buttonsPrev = buttons;
    usbUpdate = true;
  }

  // Has any axes changed state?
  if (axes != axesPrev)
  {
    Gamepad._GamepadReport.Y = ((axes & B00000100) >> 2) - ((axes & B00001000) >> 3);
    Gamepad._GamepadReport.X = ((axes & B00010000) >> 4) - (axes & B00000001);
    axesPrev = axes;
    usbUpdate = true;
  }

  // Update USB data if necessary
  if(usbUpdate)
  {
    Gamepad.send();
    usbUpdate = false;
  }
}}

void sendClock()
{
  // Send a clock pulse to pin 6 and wait
  PORTD |=  B10000000; // Enable pull-up
  delayMicroseconds(10);
  PORTD &= ~B10000000; // Disable pull-up
  delayMicroseconds(40); 
}

void startupConfig()
{
  // Read current mode from eeprom
  mode = EEPROM.read(0);
  if(mode != MODE_3BUTTON)
    mode = MODE_CD32;

  // Get time
  millisStart = millis();
  
  // Wait as long as button 1 is pressed
  while(!(PIND & B10000000)) 
  {
    if(millis() - millisStart > 5000) // Button 1 has been pressed for more than 5 seconds
    {
      // Toggle mode and save to EEPROM
      (mode == MODE_3BUTTON) ? mode = MODE_CD32 : mode = MODE_3BUTTON;
      EEPROM.update(0,mode);
      return;
    }
  }

  return;
}