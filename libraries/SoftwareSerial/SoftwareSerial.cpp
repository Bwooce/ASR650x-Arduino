/*
  This file is part of the ArduinoECCX08 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.
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
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <Arduino.h>
#include "SoftwareSerial.h"
#include "PWM1.h"

SoftwareSerial *SoftwareSerial::active_object = 0;
char SoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF];
volatile uint32_t SoftwareSerial::_receive_buffer_tail = 0;
volatile uint32_t SoftwareSerial::_receive_buffer_head = 0;
uint8_t SoftwareSerial::_bit = 0;
uint8_t SoftwareSerial::_serialState = 0;
uint8_t SoftwareSerial::_rbyte = 0;

bool SoftwareSerial::listen()
{
  if (!_rx_delay_stopbit)
    return false;

  if (active_object != this)
  {
    if (active_object)
    {
      active_object->stopListening();
    }

    _buffer_overflow = false;
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;

    if (_inverse_logic)
     	//Start bit high
      attachInterrupt(_receivePin, handle_pin_interrupt, RISING);
    else
     	//Start bit low
      attachInterrupt(_receivePin, handle_pin_interrupt, FALLING);

    return true;
  }
  return false;
}

bool SoftwareSerial::stopListening()
{
  if (active_object == this)
  {
    detachInterrupt(_receivePin);
    active_object = NULL;
    return true;
  }
  return false;
}

inline void SoftwareSerial::recv()
{
  uint8_t d = 0;
  uint8_t ints;

  if (SoftwareSerial::_serialState > 0)
  {
    return;	// we're busy in the other handler
  }
 	// If RX line is high, then we don't see any start bit
 	// so interrupt is probably not for us
  if (_inverse_logic ? rx_pin_read() : !rx_pin_read())
  {
    noInterrupts();
    _serialState = 1;	// first bit recv
    _bit = 0;
    interrupts();
    PWM1_WriteCounter(1);	// ???
    PWM1_WritePeriod(_rx_delay_centering);	// First PWM interrupt should be at centre of start bit
    PWM1_Start();
    PWM1_ISR_StartEx(SoftwareSerial::handle_timer_interrupt);
  }
}

uint32_t SoftwareSerial::rx_pin_read()
{
  return DIRECT_READ(_receiveBase, _receiveBitMask);
}

inline void SoftwareSerial::handle_pin_interrupt()
{
  if (active_object)
  {
    active_object->recv();
  }
}

// Constructor
SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /*= false */):
  _rx_delay_centering(0),
  _rx_delay_intrabit(0),
  _rx_delay_stopbit(0),
  _tx_delay(0),
  _buffer_overflow(false),
  _inverse_logic(inverse_logic),
  _debug(false)
  {
    _receivePin = receivePin;
    _transmitPin = transmitPin;
  }

// Destructor
SoftwareSerial::~SoftwareSerial()
{
  end();
}

void SoftwareSerial::setTX(uint8_t tx)
{
 	// First write, then set output. If we do this the other way around,
 	// the pin would be output low for a short while before switching to
 	// output hihg. Now, it is input with pullup for a short while, which
 	// is fine. With inverse logic, either order is fine.
  pinMode(tx, OUTPUT);
  _transmitBitMask = PIN_TO_BITMASK(tx);
  _transmitBase = PIN_TO_BASEREG(tx);
  _transmitPin = tx;
}

void SoftwareSerial::setDebugPin(uint8_t debugPin)
{
  pinMode(debugPin, OUTPUT);
  _debugBitMask = PIN_TO_BITMASK(debugPin);
  _debugBase = PIN_TO_BASEREG(debugPin);
  _debugPin = debugPin;
  _debug = true;
}

void SoftwareSerial::handle_timer_interrupt()
{

  PWM1_ClearInterrupt(PWM1_INTR_MASK_TC);
  PWM1_ClearInterrupt(PWM1_INTR_MASK_CC_MATCH);

  if (_serialState == 0)
  {
    return;
  }

  noInterrupts();
  if (SoftwareSerial::active_object->_debug)
    DIRECT_WRITE_HIGH(SoftwareSerial::active_object->_debugBase, SoftwareSerial::active_object->_debugBitMask);
  if (_bit == 0)
  {
    if (SoftwareSerial::active_object->_inverse_logic ? SoftwareSerial::active_object->rx_pin_read() : !SoftwareSerial::active_object->rx_pin_read())
    {
      PWM1_WritePeriod(SoftwareSerial::active_object->_rx_delay_intrabit);
    }
    else
    {
      _serialState = 0;
      interrupts();
      if (SoftwareSerial::active_object->_debug)
        DIRECT_WRITE_LOW(SoftwareSerial::active_object->_debugBase, SoftwareSerial::active_object->_debugBitMask);
      return;
    }
  }
  if (_bit != 0 || _bit != 9)
  {
  	// not start or stop
    _rbyte |= SoftwareSerial::active_object->rx_pin_read() << _bit - 1;
   	//(DIRECT_READ(SoftwareSerial::active_object->_receiveBase, SoftwareSerial::active_object->_receiveBitMask) ? 1 : 0) << _bit - 1;	
  }
  _bit++;
  if (_bit == 10)
  {
  	// 11th bit, so finished
   	// if buffer full, set the overflow flag and return
    uint32_t next = (SoftwareSerial::active_object->_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    if (next != SoftwareSerial::active_object->_receive_buffer_head)
    {
      if (SoftwareSerial::active_object->_inverse_logic)
      {
        _rbyte = ~_rbyte;
      }
     	// save new data in buffer: tail points to where byte goes
      SoftwareSerial::active_object->_receive_buffer[SoftwareSerial::active_object->_receive_buffer_tail] = _rbyte;	// save new byte
      SoftwareSerial::active_object->_receive_buffer_tail = next;
     	//    Serial.print(rbyte);
    }
    else
    {
      Serial.println("OVERFLOW");
      SoftwareSerial::active_object->_buffer_overflow = true;
    }
    _serialState = 0;	// ready for next byte
  }
  interrupts();
  if (SoftwareSerial::active_object->_debug)
    DIRECT_WRITE_LOW(SoftwareSerial::active_object->_debugBase, SoftwareSerial::active_object->_debugBitMask);
}

void SoftwareSerial::setRX(uint8_t rx)
{
  pinMode(rx, INPUT_PULLUP);
  _receiveBitMask = PIN_TO_BITMASK(rx);
  _receiveBase = PIN_TO_BASEREG(rx);
  _receivePin = rx;
}

void SoftwareSerial::begin(long speed)
{

  setTX(_transmitPin);
  setRX(_receivePin);
 	// Precalculate the various delays
 	//Calculate the distance between bit in timer cycles
  uint32_t bit_delay = (uint32_t)(CYDEV_BCLK__SYSCLK__HZ / PWM_DIV) / (speed);

  _tx_delay = bit_delay;
  Serial.printf("Bit delay for %d is %duS\n", speed, bit_delay);

 	//Wait 1/2 bit - (time for interrupt to be served in the first place, plus time for the timer ISR)
  _rx_delay_centering = (bit_delay / 2) - (RX_ISR_OVERHEAD + TIMER_ISR_OVERHEAD);
 	//Wait 1 bit - ISR overhead (time in each loop iteration)
  _rx_delay_intrabit = bit_delay - TIMER_ISR_OVERHEAD;
 	//Wait 1 bit (the stop one)
  _rx_delay_stopbit = bit_delay;

  _bit = 0;
  _serialState = 0;

  PWM1_SetMode(PWM1_MODE_TIMER_COMPARE);

  PWM1_Start();

  PWM1_SetPrescaler(PWM1_PRESCALE_DIVBY1);	// Probably not needed as defualt
  PWM1_SetInterruptMode(PWM1_INTR_MASK_TC);
  PWM1_ISR_ClearPending();	// ?? May not be needed
  PWM1_WritePeriod(_rx_delay_centering);
  PWM1_Stop();

  listen();
 	//  write(0x55);	//send a dummy byte to sync the start byte
}

void SoftwareSerial::end()
{
  stopListening();
}

uint32 SoftwareSerial::read()
{
  if (!isListening())
  {
    return -1;
  }

 	// Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
  {
    return -1;
  }

 	// Read from "head"
  noInterrupts();
  uint8_t d = _receive_buffer[_receive_buffer_head];	// grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  interrupts();
  return d;
}

int SoftwareSerial::available()
{
  if (!isListening())
    return 0;

  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(uint8_t b)
{
  if (_tx_delay == 0)
  {
    setWriteError();
    return 0;
  }

 	// By declaring these as local variables, the compiler will put them
 	// in registers _before_ disabling interrupts and entering the
 	// critical timing sections below, which makes it a lot easier to
 	// verify the cycle timings
  bool inv = _inverse_logic;
  uint16_t delay = _tx_delay;

  if (inv)
    b = ~b;
 	// turn off interrupts for a clean txmit
  noInterrupts();
 	// Write the start bit
  if (inv)
    DIRECT_WRITE_HIGH(_transmitBase, _transmitBitMask);
  else
    DIRECT_WRITE_LOW(_transmitBase, _transmitBitMask);

  delayMicroseconds(delay);

 	// Write each of the 8 bits
  for (uint8_t i = 0; i < 8; i++)
  {
    if (bitRead(b, i))
    {
      DIRECT_WRITE_HIGH(_transmitBase, _transmitBitMask);
    }
    else
    {
      DIRECT_WRITE_LOW(_transmitBase, _transmitBitMask);
    }
    delayMicroseconds(delay);
  }

 	// restore pin to natural state
  if (inv)
    DIRECT_WRITE_LOW(_transmitBase, _transmitBitMask);
  else
    DIRECT_WRITE_HIGH(_transmitBase, _transmitBitMask);

  interrupts();
  delayMicroseconds(delay);

  return 1;
}

void SoftwareSerial::flush()
{
  if (!isListening())
    return;

  noInterrupts();
  _receive_buffer_head = _receive_buffer_tail = 0;
  _bit = 0;
  _serialState = 0;
  interrupts();

}

int SoftwareSerial::peek()
{
  if (!isListening())
    return -1;

 	// Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

 	// Read from "head"
  return _receive_buffer[_receive_buffer_head];
}