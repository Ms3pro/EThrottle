#include "ecu_vars.h"
#include "Arduino.h"
#include "health_monitor.h"

HealthMonitor::HealthMonitor(
  EThrottleCAN * can,
  Throttle     * throttle)
 : can_(can)
 , throttle_(throttle)
{
  status_.bits.firstRun = 1u;
  status_.bits.ecuRtDataOkay = 0u;
}

void
HealthMonitor::run()
{
  const auto microsNow = micros();
  if (status_.bits.firstRun)
  {
    lastRunMicros_ = microsNow;
    status_.bits.firstRun = 0u;
    return;
  }

  const auto delta_us = microsNow - lastRunMicros_;
  microsAccum_ += delta_us;
  millisSinceLastDoChecks_ += microsAccum_ / 1000u;
  microsAccum_ %= 1000u;
  if (millisSinceLastDoChecks_ >= CHECK_INTERVAL_MS)
  {
    doChecks();
    millisSinceLastDoChecks_ = 0u;
  }
  lastRunMicros_ = microsNow;
}

void
HealthMonitor::doChecks()
{
  checkECU_RtListen();
}

void
HealthMonitor::checkECU_RtListen()
{
  // make sure we're getting at least 10Hz realtime data from megasquirt
  constexpr uint16_t EXPECTED_RT_MSGS_PER_BURST = 2u; // RtMsg00_t & RtMsg06_t
  constexpr uint16_t MIN_RT_RATE_HZ = 10u;
  constexpr uint16_t MIN_BCAST_PERIOD_MS = 1000u / MIN_RT_RATE_HZ;
  constexpr uint16_t MIN_BCAST_CYCLES_PER_CHECK = CHECK_INTERVAL_MS / MIN_BCAST_PERIOD_MS;
  constexpr uint16_t MIN_EXPECTED_RT_MSG_COUNT =
    (EXPECTED_RT_MSGS_PER_BURST * MIN_BCAST_CYCLES_PER_CHECK)
    - EXPECTED_RT_MSGS_PER_BURST; // minus 1 because it can be right on the fence sometimes
  bool okay = can_->getRtMsgCount() >= MIN_EXPECTED_RT_MSG_COUNT;
  can_->resetRtMsgCount();

  // make sure engine RPM is sane
  okay = okay && ecu::rpm >= 0u;
  okay = okay && ecu::rpm <= 20000u;

  // make sure engine idle duty is sane
  okay = okay && ecu::idleDuty >= 0u;
  okay = okay && ecu::idleDuty <= 10000u;

  status_.bits.ecuRtDataOkay = okay;
}