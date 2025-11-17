#pragma once

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
  run();

private:

};

extern TestController testController;
