/*

ReceiveOnlySoftwareSerial - adapted from ReceiveOnlySoftwareSerial by Nick Gammon 14th October 2014

SoftwareSerial.cpp (formerly NewSoftSerial.cpp) - 
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
// 
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <ReceiveOnlySoftwareSerial.h>
#include <stdlib.h>
#include <avr/io.h>

//EDIT THESE
volatile uint8_t* _DDR = &DDRB;
volatile uint8_t* _PORT = &PORTB;
volatile uint8_t* _PORTRegister=&PINB;
volatile uint8_t _PIN = PINB2;
//Look up correct PCICR bit for the correct PCINT that corresponds to the above PIN
uint8_t PCICRbit = 1; 
volatile uint8_t* PCMSK_register = &PCMSK1;  
//Look up correct PCMSK bit for the correct PCINT that corresponds to the above PIN
uint8_t PCMSKbit = 2;

 //baud rate calcs for 16Mhz look in table for these values
uint16_t _rx_delay_centering = 114;
uint16_t _rx_delay_intrabit =236;
uint16_t _rx_delay_stopbit = 236;
uint16_t _tx_delay = 233;
//END EDIT THESE

/*
//16Mhz

  //  baud    rxcenter   rxintra    rxstop    tx
  { 115200,   1,         17,        17,       12,    },
  { 57600,    10,        37,        37,       33,    },
  { 38400,    25,        57,        57,       54,    },
  { 31250,    31,        70,        70,       68,    },
  { 28800,    34,        77,        77,       74,    },
  { 19200,    54,        117,       117,      114,   },
  { 14400,    74,        156,       156,      153,   },
  { 9600,     114,       236,       236,      233,   },
  { 4800,     233,       474,       474,      471,   },
  { 2400,     471,       950,       950,      947,   },
  { 1200,     947,       1902,      1902,     1899,  },
  { 600,      1902,      3804,      3804,     3800,  },
  { 300,      3804,      7617,      7617,     7614,  },

//8Mhz
  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   1,          5,         5,      3,      },
  { 57600,    1,          15,        15,     13,     },
  { 38400,    2,          25,        26,     23,     },
  { 31250,    7,          32,        33,     29,     },
  { 28800,    11,         35,        35,     32,     },
  { 19200,    20,         55,        55,     52,     },
  { 14400,    30,         75,        75,     72,     },
  { 9600,     50,         114,       114,    112,    },
  { 4800,     110,        233,       233,    230,    },
  { 2400,     229,        472,       472,    469,    },
  { 1200,     467,        948,       948,    945,    },
  { 600,      948,        1895,      1895,   1890,   },
  { 300,      1895,       3805,      3805,   3802,   },

// 20MHz 

  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   3,          21,        21,     18,     },
  { 57600,    20,         43,        43,     41,     },
  { 38400,    37,         73,        73,     70,     },
  { 31250,    45,         89,        89,     88,     },
  { 28800,    46,         98,        98,     95,     },
  { 19200,    71,         148,       148,    145,    },
  { 14400,    96,         197,       197,    194,    },
  { 9600,     146,        297,       297,    294,    },
  { 4800,     296,        595,       595,    592,    },
  { 2400,     592,        1189,      1189,   1186,   },
  { 1200,     1187,       2379,      2379,   2376,   },
  { 600,      2379,       4759,      4759,   4755,   },
  { 300,      4759,       9523,      9523,   9520,   },
*/

ReceiveOnlySoftwareSerial *ReceiveOnlySoftwareSerial::active_object = 0;
char ReceiveOnlySoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF]; 
volatile uint8_t ReceiveOnlySoftwareSerial::_receive_buffer_tail = 0;
volatile uint8_t ReceiveOnlySoftwareSerial::_receive_buffer_head = 0;

//
// Private methods
//

/* static */ 
inline void ReceiveOnlySoftwareSerial::tunedDelay(uint16_t delay){ 
  uint8_t tmp=0;

  asm volatile("sbiw    %0, 0x01 \n\t"
    "ldi %1, 0xFF \n\t"
    "cpi %A0, 0xFF \n\t"
    "cpc %B0, %1 \n\t"
    "brne .-10 \n\t"
    : "+w" (delay), "+a" (tmp)
    : "0" (delay)
    );
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another 
uint8_t ReceiveOnlySoftwareSerial::listen(){
  if (active_object != this){
    _buffer_overflow = false;
    uint8_t oldSREG = SREG;
    cli();
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;
    SREG = oldSREG;
    return true;
  }
  return false;
}

//
// The receive routine called by the interrupt handler
//
void ReceiveOnlySoftwareSerial::recv(){

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Preserve the registers that the compiler misses
// (courtesy of Arduino forum user *etracer*)
  asm volatile(
    "push r18 \n\t"
    "push r19 \n\t"
    "push r20 \n\t"
    "push r21 \n\t"
    "push r22 \n\t"
    "push r23 \n\t"
    "push r26 \n\t"
    "push r27 \n\t"
    ::);
#endif  

  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (_inverse_logic ? rx_pin_read() : !rx_pin_read()){
    // Wait approximately 1/2 of a bit width to "center" the sample
    tunedDelay(_rx_delay_centering);

    // Read each of the 8 bits
    for (uint8_t i=0x1; i; i <<= 1){
      tunedDelay(_rx_delay_intrabit);
      uint8_t noti = ~i;
      if (rx_pin_read())
        d |= i;
      else // else clause added to ensure function timing is ~balanced
        d &= noti;
    }

    // skip the stop bit
    tunedDelay(_rx_delay_stopbit);

    if (_inverse_logic)
      d = ~d;

    // if buffer full, set the overflow flag and return
    if ((_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF != _receive_buffer_head){
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    } 
    else{
      _buffer_overflow = true;
    }
  }

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Restore the registers that the compiler misses
  asm volatile(
    "pop r27 \n\t"
    "pop r26 \n\t"
    "pop r23 \n\t"
    "pop r22 \n\t"
    "pop r21 \n\t"
    "pop r20 \n\t"
    "pop r19 \n\t"
    "pop r18 \n\t"
    ::);
#endif
}

uint8_t ReceiveOnlySoftwareSerial::rx_pin_read(){
  return *_PORTRegister & (1<<_PIN);
}

//
// Interrupt handling
//

/* static */
inline void ReceiveOnlySoftwareSerial::handle_interrupt(){
  if (active_object){
    active_object->recv();
  }
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect){
  ReceiveOnlySoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect){
  ReceiveOnlySoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect){
  ReceiveOnlySoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect){
  ReceiveOnlySoftwareSerial::handle_interrupt();
}
#endif

//
// Public methods
//

void ReceiveOnlySoftwareSerial::begin(){
  *_DDR &= ~(1 << _PIN); //  Input
  *_PORT |= 1 << _PIN; //HIGH
  // Set up RX interrupts, but only if we have a valid RX baud rate
  if (_rx_delay_stopbit){
    if (&PCICR){
      PCICR |= _BV(PCICRbit);
      *PCMSK_register |= _BV(PCMSKbit);
    }
    tunedDelay(_tx_delay); // if we were low this establishes the end
  }
  listen();
}

void ReceiveOnlySoftwareSerial::end(){
  if (PCMSK_register)
    *PCMSK_register &= ~_BV(PCMSKbit);
}

// Read data from buffer
int ReceiveOnlySoftwareSerial::read(){
  if (!isListening())
    return -1;
  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;
  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int ReceiveOnlySoftwareSerial::available(){
  if (!isListening())
    return 0;
  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

void ReceiveOnlySoftwareSerial::flush(){
  if (!isListening())
    return;
  uint8_t oldSREG = SREG;
  cli();
  _receive_buffer_head = _receive_buffer_tail = 0;
  SREG = oldSREG;
}

int ReceiveOnlySoftwareSerial::peek(){
  if (!isListening())
    return -1;
  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;
  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}
