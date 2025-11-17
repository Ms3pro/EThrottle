#include "test_controller.h"

#include <EndianUtils.h>

#include "EThrottleTables.h"
#include "Throttle.h"

TestController::TestController()
{
  outPC.status1.bits.testModeEnabled = 0u;
  outPC.status1.bits.testMode = static_cast<unsigned int>(TestModes::Idle);
  testPage.testSetpoint = 0u;
}

void
TestController::onEnableCmd(
  const bool enabled)
{
  if (enabled)
  {
    throttle.setSetpointSource(Throttle::SetpointSource::User);
    outPC.status1.bits.testModeEnabled = 1u;
    outPC.status1.bits.testMode = static_cast<unsigned int>(TestModes::Idle);
    throttle.disableThrottle();
  }
  else
  {
    throttle.setSetpointSource(Throttle::SetpointSource::PPS);
    outPC.status1.bits.testModeEnabled = 0u;
    outPC.status1.bits.testMode = static_cast<unsigned int>(TestModes::Idle);
    restoreThrottleEnableFromInhibit(throttle);
  }
}

void
TestController::onModeCmd(
  const TestModes mode)
{
  if ( ! outPC.status1.bits.testModeEnabled)
  {
    return;// ignore the command if we're not in test mode
  }

  outPC.status1.bits.testMode = static_cast<uint8_t>(mode);
  switch (mode)
  {
    case TestModes::SingleSetpoint:
      throttle.setSetpointOverride(EndianUtils::getBE(testPage.testSetpoint));
      throttle.enableThrottle();
      break;
    case TestModes::CurvePlayback:
      // TODO
      break;
    case TestModes::Idle:
    default:
      throttle.disableThrottle();
      break;
  }
}

void
TestController::run()
{
  switch (static_cast<TestModes>(outPC.status1.bits.testMode))
  {
    case TestModes::SingleSetpoint:
      throttle.setSetpointOverride(EndianUtils::getBE(testPage.testSetpoint));
      break;
    case TestModes::CurvePlayback:
      // TODO
      break;
    case TestModes::Idle:
    default:
      // nothing to do, throttle is disabled
      break;
  }
}