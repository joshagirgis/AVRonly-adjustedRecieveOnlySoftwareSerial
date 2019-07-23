/*

ReceiveOnlySoftwareSerial - adapted from ReceiveOnlySoftwareSerial by Nick Gammon 14th October 2014

SoftwareSerial.h (formerly NewSoftSerial.h) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://arduiniana.org.
*/

#ifndef ReceiveOnlySoftwareSerial_h
#define ReceiveOnlySoftwareSerial_h
#include "inttypes.h"

/******************************************************************************
* Definitions
******************************************************************************/

#define _SS_MAX_RX_BUFF 64 // RX buffer size
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif
  
class ReceiveOnlySoftwareSerial{
  private:
  // per object data
    uint8_t _transmitBitMask;
    volatile uint8_t *_transmitPortRegister;
    uint16_t _buffer_overflow:1;
    uint16_t _inverse_logic:1;

  // static data
    static char _receive_buffer[_SS_MAX_RX_BUFF]; 
    static volatile uint8_t _receive_buffer_tail;
    static volatile uint8_t _receive_buffer_head;
    static ReceiveOnlySoftwareSerial *active_object;

  // private methods
    void recv();
    uint8_t rx_pin_read();
  // private static method for timing
    static inline void tunedDelay(uint16_t delay);

  public:
  // public methods
    void begin();
    uint8_t listen();
    void end();
    uint8_t isListening() { return this == active_object; }
    uint8_t overflow() { uint8_t ret = _buffer_overflow; _buffer_overflow = 0; return ret; }
    int peek();
    int read();
    int available();
    void flush();
    static inline void handle_interrupt();
  };

#endif
