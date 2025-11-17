#pragma once

#include "EThrottleTables.h"
#include "FaultFilter.h"
#include <PID_v1.h>
#include <stdint.h>
#include <pidautotuner.h>

class Throttle
{
public:
  static constexpr uint8_t MAX_IDLE_ADDER_AUTHORITY = 20u;

  enum struct SetpointSource
  {
    PPS = 0,
    User = 1
  };

  struct FlashTableDescriptor
  {
    uint16_t xBinsFlashOffset;
    uint16_t yBinsFlashOffset;
    uint8_t nBins;
  };

  enum struct FaultClearCmd
  {
    All = 'A',
    Driver = 'd',
    PPS = 'p',
    TPS = 't',
  };

public:
  Throttle(
    const uint8_t driverPinP,
    const uint8_t driverPinN,
    const uint8_t driverPinDis,
    const uint8_t driverPinFS);

  /**
   * Configures IO pins and internal classes.
   * Normally called in setup()
   */
  void
  init(
    const uint8_t       pidSampleRate_ms,
    ThrottleOutVars_T * outVars);

  /**
   * disables the motor driver and the PID control loop.
   * sensor reading still occurs.
   */
  void
  disableThrottle();

  /**
   * enables the motor driver and the PID control loop
   */
  void
  enableThrottle();

  void
  setSetpointSource(
    const SetpointSource source);

  SetpointSource
  getSetpointSource() const;

  void
  setRangeCalPPS_A(
    const RangeCalibration & rc);

  void
  setRangeCalPPS_B(
    const RangeCalibration & rc);

  void
  setRangeCalTPS_A(
    const RangeCalibration & rc);

  void
  setRangeCalTPS_B(
    const RangeCalibration & rc);

  void
  setSensorSetup(
    const SensorSetup            setup,
    const FlashTableDescriptor & ppsCompDesc,
    const FlashTableDescriptor & tpsCompDesc,
    const uint16_t               ppsCompareThresh,
    const uint16_t               tpsCompareThresh,
    const uint16_t               tpsStall);
  
  void
  setIdleAddAuthority(
    const uint8_t idleAddAuthority);
  
  void
  setIdleAddFactor(
    const uint16_t idleAddFactor);

  /**
   * Setter for the override setpoint value.
   * @param[in] setpoint
   * the override value in percent [0 to 10000] (ie. 0 to 100%)
   */
  void
  setSetpointOverride(
    uint16_t setpoint);

  const ThrottleStatus_T &
  status() const;

  void
  clearFault(
    FaultClearCmd cmd);

  /**
   * Call this method every sample interval.
   * Ideally this method is called within a timer interrupt
   * routine.
   */
  void
  run();

  // pushes the PID coefficients into the controller
  void
  updatePID_Coeffs(
    double Kp,
    double Ki,
    double Kd);

  double
  getKp();

  double
  getKi();

  double
  getKd();

  void
  startPID_AutoTune();

  void
  stopPID_AutoTune();

private:
  void
  driverDisable();

  void
  driverEnable();

  void
  driverClearFault();

  void
  doPedal();

  void
  doThrottle();

  void
  doMotorCurrent();

private:
    const uint8_t driverPinP_;
    const uint8_t driverPinN_;
    const uint8_t driverPinDis_;
    const uint8_t driverPinFS_;

    // calibration for the pedal position sensor readings.
    // 'min' value is the ADC reading at 0% throttle
    // 'max' value is the ADC reading at 100% throttle
    RangeCalibration ppsCalA_;
    RangeCalibration ppsCalB_;

    // calibration for the throttle position sensor readings.
    // 'min' value is the ADC reading at 0% throttle
    // 'max' value is the ADC reading at 100% throttle
    RangeCalibration tpsCalA_;
    RangeCalibration tpsCalB_;
    
    // ADC readings from throttle position sensors A & B
    // range: [0 to 1023]
    uint16_t tpsA_ = 0u;
    uint16_t tpsB_ = 0u;
    // finalized throttle position based on both A & B sensor readings
    // range: [0 to 10000] (ie. 0 to 100%)
    uint16_t tps_ = 0u;

    // ADC readings from pedal position sensors A & B
    // range: [0 to 1023]
    uint16_t ppsA_ = 0u;
    uint16_t ppsB_ = 0u;
    // finalized pedal position based on both A & B sensor readings
    // range: [0 to 10000] (ie. 0 to 100%)
    uint16_t pps_ = 0u;

    // Throttle position the PID controller is targeting
    // range: [0 to 10000] (ie. 0 to 100%)
    uint16_t tpsTarget_ = 0u;

    // the minimum tps value that the engine can continue to run.
    // going below this tps value will cause the engine to stall.
    // range: [0 to 10000] (ie. 0 to 100%)
    uint16_t tpsStall_ = 0u;

    // the maximum tps % that the idle logic can add to the tps target.
    // should come from a flash table.
    // setting this to 0 will disable idle control.
    // range: [0 to MAX_IDLE_ADDER_AUTHORITY] (ie. 0=0%, 100=100%)
    uint8_t idleAddAuthority_ = 0u;

    // the requested percentage of the 'idleTpsAdderAuthority_' to
    // add to the 'tpsTarget_'
    // range: [0 to 10000] (ie. 0 to 100% of the idleTpsAdderAuthority_)
    uint16_t idleAddFactor_ = 0u;

    // portion of the tpsTarget that's coming from engine idle control.
    // range: [0 to 10000] (ie. 0 to 100%)
    uint16_t idleAdder_ = 0u;

    // portion of the tpsTarget that's coming from the accelerator pedal
    // ppsAdder = ((10000 - tpsStall_ - idleAdder) * pps) / 10000
    // range: [0 to 10000] (ie. 0 to 100%)
    uint16_t ppsAdder_ = 0u;

    // PWM motor driver output
    // range: [-255 to 255] if in h-bridge mode (negative means reverse)
    //        [0 to 255] if in normal mode
    int16_t motorOut_ = 0u;
    
    // ADC readings from motor driver current feedback pin
    // range: [0 to 1023]
    uint16_t driverFB_ = 0u;

    // Motor current calculated based on driver feedback pin voltage
    // range: [0 to 65535] in milliamps
    uint16_t motorCurrent_mA_ = 0u;

    // RAM variables
    ThrottleOutVars_T * outVars_ = nullptr;

    // status maintained in RAM
    ThrottleStatus_T status_;

    // rate at which the PID algorith runs in milliseconds
    uint8_t pidSampleRate_ms_ = 100u;

    // P,I, and D coefficient fed into the PID controller
    // to update this settings, call updatePID_Coeffs()
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;

    // current real-world TPS reading fed into the PID controller
    double pidIn_ = 0.0;
    // PID's requested PWM for motor controler
    // Range: [-255 to +255] for h-bridge driver (allows reverse)
    //        [0 to +255] for normal driver
    double pidOut_ = 0.0;
    // target TPS position fed into PID controller
    double pidSetpoint_ = 0.0;
    // the PID controller itself
    PID pid_;

    PIDAutotuner tuner_;

    SetpointSource setpointSource_ = SetpointSource::PPS;

    // user specified setpoint via setSetpointOverride()
    // range: [0 to 10000] (ie. 0% to 100%)
    uint16_t userSetpoint_ = 0u;

    SensorSetup sensorSetup_;
    FlashTableDescriptor ppsCompDesc_;
    FlashTableDescriptor tpsCompDesc_;
    // threshold used to compare the absolute value of the ADC
    // delta in the sensor comparison logic.
    uint16_t ppsCompareThresh_ = 20u;
    uint16_t tpsCompareThresh_ = 20u;
    // if pps/tps safety comparison is enabled, this is the delta between
    // what we expect the readings to be vs. the actual. if these values
    // exceed 'ppsComparThresh_' or 'tpsCompareThresh_' repsectively, then
    // a fault is triggered.
    // range: [-2048 to 2048] (ie. 0% to 100%)
    int16_t ppsDelta_ = 0u;
    int16_t tpsDelta_ = 0u;
    uint8_t ppsCompFaultCount_ = 0u;
    uint8_t tpsCompFaultCount_ = 0u;
    FaultFilter ppsFaultFilter_;
    FaultFilter tpsFaultFilter_;

    uint8_t driverFaultCount_ = 0u;

};

extern Throttle throttle;

// accessor utilities
void
loadFlashPage1ToThrottle(
  Throttle &throttle);

void
storeThrottlePID_ToFlash(
  Throttle &throttle);

void
restoreThrottleEnableFromInhibit(
  Throttle &throttle);
