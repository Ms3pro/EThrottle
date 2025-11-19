#pragma once

#include <stdint.h>

enum struct TestModes {
  Idle = 0,
  SingleSetpoint = 1,
  CurvePlayback = 2
};

class TestController
{
public:
  TestController();

  void
  onEnableCmd(
    const bool enabled);

  void
  onModeCmd(
    const TestModes mode);

  // call this in the main loop
  void
  run(
    const uint16_t dt_usec);

private:
  void
  tryPlaybackStart();

  void
  playbackRun(
    const uint16_t dt_usec);

private:
  uint16_t accum_usec_ = 0u;// time accumulator
  uint16_t playTimeOffset_ms_ = 0u;
  uint16_t maxPlayTime_ms_ = 0u;
  uint8_t playCountsRemaining_ = 0u;

};

extern TestController testController;
