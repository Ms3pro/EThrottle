#include "Throttle.h"

#include <Arduino.h>
#include <logging.h>

#include "adc_ctrl.h"
#include "config.h"
#include "EThrottleTables.h"
#include <EndianUtils.h>
#include <FlashUtils.h>

#define ST_FAULT_TIMEOUT_MAX 10
#define LT_FAULT_TIMEOUT_MAX 100
#define LT_FAULT_THRESH 5

// toggle a pin on different fault conditions
#define ENABLE_ETHROTTLE_STROBES  0 // global enable
#define STROBE_PIN A4
#define STROBE_ON_TPS_FAULT 1
#define STROBE_ON_PPS_FAULT 0

Throttle::Throttle(
    const uint8_t driverPinP,
    const uint8_t driverPinN,
    const uint8_t driverPinDis,
    const uint8_t driverPinFS)
 : driverPinP_(driverPinP)
 , driverPinN_(driverPinN)
 , driverPinDis_(driverPinDis)
 , driverPinFS_(driverPinFS)
 , pid_(&pidIn_,&pidOut_,&pidSetpoint_,0,0,0, DIRECT)
{
  // setup ADC measurements
  adc::tpsA.adcMUX = 0;// A0
  adc::tpsA.mMode = adc::MeasurementMode::Continuous;
  adc::tpsB.adcMUX = 1;// A1
  adc::tpsB.mMode = adc::MeasurementMode::Continuous;
  adc::ppsA.adcMUX = 2;// A2
  adc::ppsA.mMode = adc::MeasurementMode::Continuous;
  adc::ppsB.adcMUX = 3;// A3
  adc::ppsB.mMode = adc::MeasurementMode::Continuous;
  adc::driverFB.adcMUX = 5;// A5
  adc::driverFB.mMode = adc::MeasurementMode::OneShot;
  // Rationale for triggering current feedback off timer0 overflow:
  //
  // The motor pins are driven by PD6 (OC0A) and PD5 (OC0B).
  // Those pins are driven by the timer0 PWM logic.
  // Here's the timer 0 config set by Arduino's wiring library.
  // TCCR0A: 0x83
  // TCCR0B: 0x03
  //   WGM   -> 0x011 (Fast PWM mode)
  //   COM0A -> 0b10 (clear OCA on compare match, set OCA at BOTTOM)
  //   COM0B -> 0b10 (clear OCB on compare match, set OCB at BOTTOM)
  //   CS    -> 0b011 (Clk_io / 64)
  // Motor driver measurements must we precisely taken when the PWM
  // outputs are being driven high; otherwise, the readings are too
  // random. Since both PWM outputs are driven high when timer0 hits
  // BOTTOM (ie. overflows), we can auto trigger the current feedback
  // ADC measurement off of the timer0 overflow event.
  adc::driverFB.tMode = adc::TriggerMode::Tmr0_Ovrf;

  status_.pidAutoTuneBusy = 0;
  status_.ppsComparisonFault = 0;
  status_.tpsComparisonFault = 0;
  status_.throttleEnabled = 0;
  status_.motorEnabled = 0;
  status_.motorDriverFault = 0;
  status_.adcStalled = 0;

  // zero out coefficients to be safe
  updatePID_Coeffs(0.0,0.0,0.0);

  // defaults. these can get changed via setSensorSetup()
  sensorSetup_.comparePPS = 0;
  sensorSetup_.preferPPS_A = 1;
  sensorSetup_.compareTPS = 0;
  sensorSetup_.preferTPS_A = 0;
}

void
Throttle::init(
  const uint8_t       pidSampleRate_ms,
  ThrottleOutVars_T * outVars)
{
  pidSampleRate_ms_ = pidSampleRate_ms;
  outVars_ = outVars;

  pinMode(driverPinP_, OUTPUT);
  pinMode(driverPinN_, OUTPUT);
  pinMode(driverPinDis_, OUTPUT);
  pinMode(driverPinFS_, INPUT_PULLUP);
#if ENABLE_ETHROTTLE_STROBES
    pinMode(STROBE_PIN, OUTPUT);
#endif

  disableThrottle();
  analogWrite(driverPinP_,0);
  analogWrite(driverPinN_,0);

  // setup the PID controller class
  pid_.SetMode(AUTOMATIC);
#ifdef SUPPORT_H_BRIDGE
  // negative range is used to reverse motor to close throttle
  pid_.SetOutputLimits(-255, 255);
  tuner_.setOutputRange(-255, 255);
#else
  pid_.SetOutputLimits(0, 255);
  tuner_.setOutputRange(0, 255);
#endif

  pid_.SetSampleTime(pidSampleRate_ms_);
  tuner_.setZNMode(PIDAutotuner::ZNModeNoOvershoot);
  tuner_.setLoopInterval(pidSampleRate_ms_ * 1000);
}

void
Throttle::disableThrottle()
{
  status_.throttleEnabled = 0;
  driverDisable();
}

void
Throttle::enableThrottle()
{
  status_.throttleEnabled = 1;
  driverEnable();
}

void
Throttle::setSetpointSource(
  const SetpointSource source)
{
  setpointSource_ = source;
}

Throttle::SetpointSource
Throttle::getSetpointSource() const
{
  return setpointSource_;
}

void
Throttle::setRangeCalPPS_A(
  const RangeCalibration & rc)
{
  ppsCalA_ = rc;
}

void
Throttle::setRangeCalPPS_B(
  const RangeCalibration & rc)
{
  ppsCalB_ = rc;
}

void
Throttle::setRangeCalTPS_A(
  const RangeCalibration & rc)
{
  tpsCalA_ = rc;
}

void
Throttle::setRangeCalTPS_B(
  const RangeCalibration & rc)
{
  tpsCalB_ = rc;
}

void
Throttle::setSensorSetup(
  const SensorSetup            setup,
  const FlashTableDescriptor & ppsCompDesc,
  const FlashTableDescriptor & tpsCompDesc,
  const uint16_t               ppsCompareThresh,
  const uint16_t               tpsCompareThresh,
  const uint16_t               tpsStall)
{
  sensorSetup_ = setup;
  ppsCompDesc_ = ppsCompDesc;
  tpsCompDesc_ = tpsCompDesc;
  ppsCompareThresh_ = ppsCompareThresh;
  tpsCompareThresh_ = tpsCompareThresh;
  tpsStall_ = tpsStall;
}
  
void
Throttle::setIdleAddAuthority(
  const uint8_t idleAddAuthority)
{
  idleAddAuthority_ = min(idleAddAuthority, MAX_IDLE_ADDER_AUTHORITY);
}

void
Throttle::setIdleAddFactor(
  const uint16_t idleAddFactor)
{
  idleAddFactor_ = min(idleAddFactor, 10000u);
}

void
Throttle::setSetpointOverride(
  uint16_t setpoint)
{
  if (setpoint > 10000)
  {
    setpoint = 10000;
  }
  userSetpoint_ = setpoint;
}

const ThrottleStatus_T &
Throttle::status() const
{
  return status_;
}

void
Throttle::clearFault(
  Throttle::FaultClearCmd cmd)
{
  const bool clrAll = cmd == FaultClearCmd::All;

  if (clrAll || cmd == FaultClearCmd::PPS)
  {
    status_.ppsComparisonFault = 0;
    ppsFaultFilter_.reset();
  }
  if (clrAll || cmd == FaultClearCmd::TPS)
  {
    status_.tpsComparisonFault = 0;
    tpsFaultFilter_.reset();
  }
  if (clrAll || cmd == FaultClearCmd::Driver)
  {
    driverClearFault();
  }
}

void
Throttle::run()
{
  doPedal();
  doThrottle();
  doMotorCurrent();

  DEBUG("driverFB: %4d; motorCurrent: %4d mA", driverFB_, motorCurrent_mA_);

  if (outVars_)
  {
    EndianUtils::setBE(outVars_->tpsA, tpsA_);
    EndianUtils::setBE(outVars_->tpsB, tpsB_);
    EndianUtils::setBE(outVars_->tps, tps_);
    EndianUtils::setBE(outVars_->ppsA, ppsA_);
    EndianUtils::setBE(outVars_->ppsB, ppsB_);
    EndianUtils::setBE(outVars_->pps, pps_);
    EndianUtils::setBE(outVars_->tpsTarget, tpsTarget_);
    EndianUtils::setBE(outVars_->motorOut, motorOut_);
    EndianUtils::setBE(outVars_->motorCurrent_mA, motorCurrent_mA_);
    outVars_->status = status_;
    EndianUtils::setBE(outVars_->idleAdder, idleAdder_);
    EndianUtils::setBE(outVars_->ppsAdder, ppsAdder_);
    EndianUtils::setBE(outVars_->driverFB, driverFB_);
    EndianUtils::setBE(outVars_->ppsSafetyDelta, ppsDelta_);
    EndianUtils::setBE(outVars_->tpsSafetyDelta, tpsDelta_);
    outVars_->ppsCompFaultCount = ppsCompFaultCount_;
    outVars_->tpsCompFaultCount = tpsCompFaultCount_;
    outVars_->driverFaultCount = driverFaultCount_;
  }
}

void
Throttle::updatePID_Coeffs(
  double Kp,
  double Ki,
  double Kd)
{
  pid_.SetTunings(Kp,Ki,Kd);
}

double
Throttle::getKp()
{
  return pid_.GetKp();
}

double
Throttle::getKi()
{
  return pid_.GetKi();
}

double
Throttle::getKd()
{
  return pid_.GetKd();
}

void
Throttle::startPID_AutoTune()
{
  if (status_.pidAutoTuneBusy) {
    return;
  }

  const uint16_t AUTOTUNE_TARGET = 5000;// 50%

  status_.pidAutoTuneBusy = 1;
  tuner_.setTargetInputValue(AUTOTUNE_TARGET);
  tuner_.startTuningLoop(micros());
}

void
Throttle::stopPID_AutoTune()
{
  status_.pidAutoTuneBusy = 0;
}

void
Throttle::driverDisable()
{
  digitalWrite(driverPinDis_, 1);
  status_.motorEnabled = 0;
}

void
Throttle::driverEnable()
{
  if (status_.throttleEnabled)
  {
    digitalWrite(driverPinDis_, 0);
    status_.motorEnabled = 1;
  }
}

void
Throttle::driverClearFault()
{
  // MC33887 fault status is sticky
  // need to disable/enable motor to clear
  driverDisable();
  driverEnable();
}

void
Throttle::doPedal()
{
  ppsA_ = adc::ppsA.value;
  ppsB_ = adc::ppsB.value;

  // normalize PPS readings based on calibrated min/max values
  const auto ppsA_Norm = static_cast<uint16_t>(map(ppsA_, ppsCalA_.min, ppsCalA_.max, 0, 10000));
  const auto ppsB_Norm = static_cast<uint16_t>(map(ppsB_, ppsCalB_.min, ppsCalB_.max, 0, 10000));

  // safety check the raw ADC values
  if (sensorSetup_.comparePPS)
  {
    const auto preferADC = (sensorSetup_.preferPPS_A ? ppsA_ : ppsB_);
    const auto otherADC = (sensorSetup_.preferPPS_A ? ppsB_ : ppsA_);

    // lookup what we expect the other sensor's ADC value to be based
    // on the prefered sensor's reading.
    const auto otherExpected = FlashUtils::lerpU16(
      ppsCompDesc_.xBinsFlashOffset,
      ppsCompDesc_.yBinsFlashOffset,
      ppsCompDesc_.nBins,
      preferADC);
    ppsDelta_ = otherADC - otherExpected;

    // fault filtering logic
    const bool faulted = abs(ppsDelta_) >= ppsCompareThresh_;
#if ENABLE_ETHROTTLE_STROBES && STROBE_ON_PPS_FAULT
      digitalWrite(STROBE_PIN, faulted);
      digitalWrite(STROBE_PIN, 0);
#endif
    ppsCompFaultCount_ += faulted;
    const ModeWithTransition mwt = ppsFaultFilter_.process(faulted);
    if (mwt.mode == FaultMode::LongTerm)
    {
      pps_ = 0;// default to 0% pedal position to be safe
      status_.ppsComparisonFault = 1;
    }
    else if (mwt.mode == FaultMode::Nominal)
    {
      status_.ppsComparisonFault = 0;
    }
  }

  // compute final pps values based on optional fault filtering logic
  // Note: short term fault will preserve previous good PPS value
  if (status_.ppsComparisonFault == 0)
  {
    // Compute PPS percentage based on prefered sensor's normalized
    // percentage. The tuner should set the prefered sensor (A or B)
    // based on which one gives readings over the full range of the
    // accelerator pedal.
    // Note: pps_ can get set to 0% if PPS safety checks fail.
    pps_ = (sensorSetup_.preferPPS_A ? ppsA_Norm : ppsB_Norm);
    if (pps_ > 10000u) {
      pps_ = 10000u;
    }
  }
}

void
Throttle::doThrottle()
{
  tpsA_ = adc::tpsA.value;
  tpsB_ = adc::tpsB.value;

  // normalize TPS readings based on calibrated min/max values
  const auto tpsA_Norm = static_cast<uint16_t>(map(tpsA_, tpsCalA_.min, tpsCalA_.max, 0, 10000));
  const auto tpsB_Norm = static_cast<uint16_t>(map(tpsB_, tpsCalB_.min, tpsCalB_.max, 0, 10000));

  // safety check the raw ADC values
  if (sensorSetup_.compareTPS)
  {
    const auto preferADC = (sensorSetup_.preferTPS_A ? tpsA_ : tpsB_);
    const auto otherADC = (sensorSetup_.preferTPS_A ? tpsB_ : tpsA_);

    // lookup what we expect the other sensor's ADC value to be based
    // on the prefered sensor's reading.
    const auto otherExpected = FlashUtils::lerpU16(
      tpsCompDesc_.xBinsFlashOffset,
      tpsCompDesc_.yBinsFlashOffset,
      tpsCompDesc_.nBins,
      preferADC);
    tpsDelta_ = otherADC - otherExpected;

    // fault filtering logic
    const bool faulted = abs(tpsDelta_) >= tpsCompareThresh_;
#if ENABLE_ETHROTTLE_STROBES && STROBE_ON_TPS_FAULT
      digitalWrite(STROBE_PIN, faulted);
      digitalWrite(STROBE_PIN, 0);
#endif
    tpsCompFaultCount_ += faulted;
    const ModeWithTransition mwt = tpsFaultFilter_.process(faulted);
    if (mwt.mode == FaultMode::LongTerm)
    {
      driverDisable();
      status_.tpsComparisonFault = 1;
    }
    else if (mwt.mode == FaultMode::Nominal && mwt.transition)
    {
      driverEnable();
      status_.tpsComparisonFault = 0;
    }
  }

  // compute final tps values based on if we're within fault
  // Note: short term fault will preserve previous good TPS value
  if (status_.tpsComparisonFault == 0)
  {
    // Compute TPS percentage based on prefered sensor's normalized
    // percentage. The tuner should set the prefered sensor (A or B)
    // based on which one gives readings over the full range of the
    // throttle blade.
    // Note: tps_ can get set to 0% if TPS safety checks fail.
    tps_ = (sensorSetup_.preferTPS_A ? tpsA_Norm : tpsB_Norm);
  }

  // update PID controller
  bool newCycle = 0;
  if (status_.throttleEnabled)
  {
    switch (setpointSource_)
    {
      case SetpointSource::PPS:
      {
        idleAdder_ = static_cast<uint32_t>(idleAddAuthority_) * idleAddFactor_ / 100u;
        ppsAdder_ = (static_cast<int32_t>(10000 - tpsStall_ - idleAdder_) * pps_) / 10000;
        tpsTarget_ = tpsStall_ + idleAdder_ + ppsAdder_;
        break;
      }
      case SetpointSource::User:
        tpsTarget_ = userSetpoint_;
        break;
    }

    // clamp the tpsTarget_ within tpsStall_ and 100%
    if (tpsTarget_ > 11000u)
    {
      // >110% probably means we underflowed, so clamp to tpsStall_
      tpsTarget_ = tpsStall_;
    }
    else if (tpsTarget_ > 10000u)
    {
      tpsTarget_ = 10000u;
    }

    pidSetpoint_ = static_cast<double>(tpsTarget_);
    pidIn_ = static_cast<double>(tps_);
    newCycle = pid_.Compute();

    // handle PID auto-tune logic
    if (status_.pidAutoTuneBusy) {
      pidOut_ = tuner_.tunePID(pidIn_, micros());

      if (tuner_.isFinished()) {
        stopPID_AutoTune();

        INFO("auto tune done!");
        INFO(
          "Kp: %d, Ki: %d, Kd: %d",
          (uint16_t)(tuner_.getKp() * 100),
          (uint16_t)(tuner_.getKi() * 100),
          (uint16_t)(tuner_.getKd() * 100));
      }
    }

    // negative PWM means we need to close throttle; results in inverse
    // motor polarity in H-Bridge driver use cases, or just undriven
    // motor otherwise (relies on throttle body return spring to close)
    motorOut_ = static_cast<int16_t>(pidOut_);
  }
  else
  {
    tpsTarget_ = 0u;
    motorOut_ = 0u;
    motorCurrent_mA_ = 0u;
  }

  // make sure ADC conversions are running correctly
  if (newCycle)
  {
    static uint16_t prevADC_Cycles = 0;
    const uint16_t deltaCycles = adc::conversionCycles - prevADC_Cycles;
    prevADC_Cycles = adc::conversionCycles;
    if (deltaCycles == 0)
    {
      status_.adcStalled = 1;
      driverDisable();
    }
  }

  // update motor driver fault status
  // The MC33887 fault status pin is active low. It will assert the fault status
  // pin if the driver is in the disabled state. Since we sometimes disable the
  // motor driver on purpose, I'm checking that we intentionally have the motor
  // in the enabled state before flagging a fault.
  const bool liveMotorDriverFault = status_.motorEnabled && digitalRead(driverPinFS_) == 0;
  if (liveMotorDriverFault)
  {
    driverClearFault();
    if (status_.motorDriverFault == 0)
    {
      // only increment counter upon entering fault condition
      driverFaultCount_++;
    }
  }
  status_.motorDriverFault = liveMotorDriverFault;

  // drive motor outputs
#ifdef SUPPORT_H_BRIDGE
  // handle h-bridge polarity inversion logic
  if (motorOut_ > 0) {
    analogWrite(driverPinP_, motorOut_);
    analogWrite(driverPinN_, 0);
    // setup motor current measurement on OC0B match ISR (motor P is on pin 5 - OC0B)
    adc::driverFB.tMode = adc::TriggerMode::ISR_Tmr0_OCB;
  } else if (motorOut_ < 0) {
    analogWrite(driverPinP_, 0);
    analogWrite(driverPinN_, motorOut_ * -1);// * -1 to write PWM magnitude only
    // setup motor current measurement on OC0A match ISR (motor N is on pin 6 - OC0A)
    adc::driverFB.tMode = adc::TriggerMode::ISR_Tmr0_OCA;
  } else {
    analogWrite(driverPinP_, 0);
    analogWrite(driverPinN_, 0);
  }
#else
  // no h-bridge means we can only drive the motor 1 way
  analogWrite(driverPinP_, motorOut_);
  analogWrite(driverPinN_, 0);
  // setup motor current measurement on OC0B match ISR (motor P is on pin 5 - OC0B)
  adc::driverFB.tMode = adc::TriggerMode::ISR_Tmr0_OCB;
#endif

  // request driver current ADC measurement once a PID cycle
  adc::driverFB.flags.needsMeasure = newCycle;
}

/**
 * @param[inout] value the value to smooth
 * @param[in] newVal the new value to smooth towards
 * @param[in] smoothFactor factor to smooth by. 1 = high smoothed, 100 = no smoothing
 */
void
smoothU16(
  uint16_t     & value,
  const uint16_t newVal,
  const uint8_t  smoothFactor)
{
  value += static_cast<int16_t>((smoothFactor * (static_cast<int32_t>(newVal) - value)) / 100u);
}

void
Throttle::doMotorCurrent()
{
  // only process value when sample is new (smoothing algorithm doesn't work well)
  if (adc::driverFB.flags.sampled)
  {
    adc::driverFB.flags.sampled = 0;// clear flag

    driverFB_ = adc::driverFB.value;

    // feedback pin drives 1/375th the current to ground through 100ohm resistor.
    // V_fb = ADC * 5.0v / 1023
    // V_fb = I * R = I_fb * 100ohm
    // I_fb = I_m * 1/375
    //  V_fb -> voltage see over 100ohm resistor
    //  I_fb -> current out of driver's feedback pin (proportional to I_m)
    //  I_m  -> current through motor
    // solve for 'I_m' using the above equations:
    //  I_m = ADC * (5/1023) * (375/100)
    //  I_m = ADC * 0.018328 <- in amps
    const auto motorCurrentNow = static_cast<uint16_t>(driverFB_ * 18.328f);

    // apply smoothing
    smoothU16(motorCurrent_mA_, motorCurrentNow, 10u);
  }
}

void
loadFlashPage1ToThrottle(
  Throttle &throttle)
{
  // misc. control params
  restoreThrottleEnableFromInhibit(throttle);
  throttle.setIdleAddAuthority(EEPROM.read(FIELD_OFFSET_CFG_PAGE1(idleAddAuthority)));

  // PID coefs.
  throttle.updatePID_Coeffs(
    FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(throttleKp)) / 100.0,
    FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(throttleKi)) / 100.0,
    FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(throttleKd)) / 100.0);

  // sensor range calibrations
  RangeCalibration rc;
  rc.min = FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(ppsCalA.min));
  rc.max = FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(ppsCalA.max));
  throttle.setRangeCalPPS_A(rc);
  rc.min = FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(ppsCalB.min));
  rc.max = FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(ppsCalB.max));
  throttle.setRangeCalPPS_B(rc);
  rc.min = FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(tpsCalA.min));
  rc.max = FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(tpsCalA.max));
  throttle.setRangeCalTPS_A(rc);
  rc.min = FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(tpsCalB.min));
  rc.max = FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(tpsCalB.max));
  throttle.setRangeCalTPS_B(rc);

  // load sensor setup
  SensorSetupUnion setupU;
  setupU.word = EEPROM.read(FIELD_OFFSET_CFG_PAGE1(sensorSetup.word));

  Throttle::FlashTableDescriptor ppsFTD;
  ppsFTD.xBinsFlashOffset = FIELD_OFFSET_CFG_PAGE1(ppsCompCurve_xBins);
  ppsFTD.yBinsFlashOffset = FIELD_OFFSET_CFG_PAGE1(ppsCompCurve_yBins);
  ppsFTD.nBins = SENSOR_COMPARE_CURVE_N_BINS;

  Throttle::FlashTableDescriptor tpsFTD;
  tpsFTD.xBinsFlashOffset = FIELD_OFFSET_CFG_PAGE1(tpsCompCurve_xBins);
  tpsFTD.yBinsFlashOffset = FIELD_OFFSET_CFG_PAGE1(tpsCompCurve_yBins);
  tpsFTD.nBins = SENSOR_COMPARE_CURVE_N_BINS;

  throttle.setSensorSetup(
    setupU.bits,
    ppsFTD,
    tpsFTD,
    FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(ppsCompareThresh)),
    FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(tpsCompareThresh)),
    FlashUtils::readBE<uint16_t>(FIELD_OFFSET_CFG_PAGE1(tpsStall)));
}

void
storeThrottlePID_ToFlash(
  Throttle &throttle)
{
  FlashUtils::writeBE(FIELD_OFFSET_CFG_PAGE1(throttleKp), (uint16_t)(throttle.getKp() * 100.0));
  FlashUtils::writeBE(FIELD_OFFSET_CFG_PAGE1(throttleKi), (uint16_t)(throttle.getKi() * 100.0));
  FlashUtils::writeBE(FIELD_OFFSET_CFG_PAGE1(throttleKd), (uint16_t)(throttle.getKd() * 100.0));
}

void
restoreThrottleEnableFromInhibit(
  Throttle &throttle)
{
  ThrottleControl tCtrl;
  tCtrl.word = EEPROM.read(FIELD_OFFSET_CFG_PAGE1(throttleCtrl));
  if (tCtrl.bits.throttleInhibit)
  {
    throttle.disableThrottle();
  }
  else
  {
    throttle.enableThrottle();
  }
}
