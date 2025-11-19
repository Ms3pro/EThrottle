#include "test_controller.h"

#include <EndianUtils.h>

#include "EThrottleTables.h"
#include "Throttle.h"

TestController::TestController()
{
  outPC.status1.bits.testModeEnabled = 0u;
  outPC.status1.bits.testMode = static_cast<unsigned int>(TestModes::Idle);
  testPage.testSetpoint = 0u;

  // load a sine wave into the test playback curve
  EndianUtils::setBE(testPage.testPlaybackCurve_xBins[0 ],    0u);
  EndianUtils::setBE(testPage.testPlaybackCurve_xBins[1 ],  273u);
  EndianUtils::setBE(testPage.testPlaybackCurve_xBins[2 ],  545u);
  EndianUtils::setBE(testPage.testPlaybackCurve_xBins[3 ],  818u);
  EndianUtils::setBE(testPage.testPlaybackCurve_xBins[4 ], 1091u);
  EndianUtils::setBE(testPage.testPlaybackCurve_xBins[5 ], 1364u);
  EndianUtils::setBE(testPage.testPlaybackCurve_xBins[6 ], 1636u);
  EndianUtils::setBE(testPage.testPlaybackCurve_xBins[7 ], 1909u);
  EndianUtils::setBE(testPage.testPlaybackCurve_xBins[8 ], 2182u);
  EndianUtils::setBE(testPage.testPlaybackCurve_xBins[9 ], 2455u);
  EndianUtils::setBE(testPage.testPlaybackCurve_xBins[10], 2727u);
  EndianUtils::setBE(testPage.testPlaybackCurve_xBins[11], 3000u);
  testPage.testPlaybackCurve_yBins[0 ] =   50;
  testPage.testPlaybackCurve_yBins[1 ] =   77;
  testPage.testPlaybackCurve_yBins[2 ] =   95;
  testPage.testPlaybackCurve_yBins[3 ] =   99;
  testPage.testPlaybackCurve_yBins[4 ] =   88;
  testPage.testPlaybackCurve_yBins[5 ] =   64;
  testPage.testPlaybackCurve_yBins[6 ] =   36;
  testPage.testPlaybackCurve_yBins[7 ] =   12;
  testPage.testPlaybackCurve_yBins[8 ] =    1;
  testPage.testPlaybackCurve_yBins[9 ] =    5;
  testPage.testPlaybackCurve_yBins[10] =   23;
  testPage.testPlaybackCurve_yBins[11] =   50;
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
      tryPlaybackStart();
      break;
    case TestModes::Idle:
    default:
      throttle.disableThrottle();
      break;
  }
}

void
TestController::run(
    const uint16_t dt_usec)
{
  switch (static_cast<TestModes>(outPC.status1.bits.testMode))
  {
    case TestModes::SingleSetpoint:
      throttle.setSetpointOverride(EndianUtils::getBE(testPage.testSetpoint));
      break;
    case TestModes::CurvePlayback:
      playbackRun(dt_usec);
      break;
    case TestModes::Idle:
    default:
      // nothing to do, throttle is disabled
      break;
  }
}

void
TestController::tryPlaybackStart()
{
    // process & validate the playback curve. we're finding the max time
    // offset specified, and making sure that the time offsets are ascending.
    uint16_t prevOffset_ms = 0u;
    maxPlayTime_ms_ = 0u;
    outPC.status1.bits.testModeBadCurve = 0u;
    for (uint8_t i=0; i<TEST_PLAYBACK_CURVE_N_BINS; i++)
    {
      const auto offset_ms = EndianUtils::getBE(testPage.testPlaybackCurve_xBins[i]);

      // make sure time offsets are ascending
      if (i > 0 && offset_ms < prevOffset_ms)
      {
        outPC.status1.bits.testModeBadCurve = 1u;
      }

      if (offset_ms > maxPlayTime_ms_)
      {
        maxPlayTime_ms_ = offset_ms;
      }

      prevOffset_ms = offset_ms;
    }

    // if the playback curve is bad, then revert to Idle mode and don't start
    if (outPC.status1.bits.testModeBadCurve)
    {
      onModeCmd(TestModes::Idle);
      return;
    }

    accum_usec_ = 0u;
    playTimeOffset_ms_ = 0u;
    playCountsRemaining_ = testPage.testPlaybackCount;
    throttle.setSetpointOverride(static_cast<uint16_t>(testPage.testPlaybackCurve_yBins[0]) * 100u);
    throttle.enableThrottle();
}

void
TestController::playbackRun(
  const uint16_t dt_usec)
{
  accum_usec_ += dt_usec;
  const uint16_t accum_ms = accum_usec_ / 1000u;
  accum_usec_ -= accum_ms * 1000u;

  playTimeOffset_ms_ += accum_ms;
  if (playTimeOffset_ms_ > maxPlayTime_ms_)
  {
    playTimeOffset_ms_ -= maxPlayTime_ms_;

    // update playback counter. if not running idefinitely, then
    // stop once all playback counts are completed.
    if (testPage.testPlaybackCount != 0)
    {
      playCountsRemaining_--;
      if (playCountsRemaining_ == 0)
      {
        onModeCmd(TestModes::Idle);// stop
      }
    }
  }
}