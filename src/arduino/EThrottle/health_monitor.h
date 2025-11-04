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
  run();

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

  // micros() values at the time of last run() call
  uint32_t lastRunMicros_ = 0u;

  uint16_t microsAccum_ = 0u;

  // milliseconds since last doChecks() call
  uint16_t millisSinceLastDoChecks_ = 0u;

};