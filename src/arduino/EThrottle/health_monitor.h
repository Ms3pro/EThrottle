#pragma once

#include <stdint.h>

#include "can.h"
#include "Throttle.h"

class HealthMonitor
{
public:
  static constexpr uint16_t CHECK_INTERVAL_MS = 1000u;

  union Status {
    Status()
    {
      word = 0u;
    }

    struct Bits {
      uint8_t firstRun      : 1;
      uint8_t ecuRtDataOkay : 1;
    } bits;
    uint8_t word;
  };

  HealthMonitor(
    EThrottleCAN * can,
    Throttle     * throttle);

  void
  run(
    const uint16_t dt_usec);

  inline const Status & getStatus() const {return status_;}

private:
  void
  doChecks();

  void
  checkECU_RtListen();

private:
  EThrottleCAN * can_ = nullptr;
  Throttle * throttle_ = nullptr;

  Status status_;

  uint16_t accum_usec_ = 0u;// time accumulator

  // milliseconds since last doChecks() call
  uint16_t millisSinceLastDoChecks_ = 0u;

};