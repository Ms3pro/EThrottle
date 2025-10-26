#pragma once

#include <stdint.h>

// ECU variables received via Megasquirt real-time broadcast data
namespace ecu
{
  inline uint16_t seconds = 0u;  // ellapsed seconds since power-on
  inline uint16_t rpm = 0u;      // engine RPM
  inline uint16_t idleDuty = 0u; // megasquirt's total idle control duty cycle (0-10000 ie. 0-100% depending on min/max flash in the ECU)
}