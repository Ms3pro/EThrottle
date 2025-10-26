#pragma once

#include <stdint.h>

namespace adc
{

  enum struct MeasurementMode
  {
    // will not measure this entry at all
    Disabled,
    // measures the entry every iteration
    Continuous,
    // measures the entry if the 'needsMeasure' flag is set
    OneShot
  };

  // specify how the ADC should be started once an entry becomes
  // ready for scheduling.
  // Note: values correspond to ADC Auto Trigger Source values (reg ADCSRB)
  enum struct TriggerMode
  {
    Immediate = 0,
    Tmr0_MatchA = 3,
    Tmr0_Ovrf = 4,
    Tmr1_MatchB = 5,
    Tmr1_Ovrf = 6,
    Tmr1_CapEvt = 7,

    ISR_Tmr0_OCA = 20,
    ISR_Tmr0_OCB = 21
  };

  enum struct ADC_State
  {
    Stopped = 0,
    Started = 1,
    PendingTrigger = 2,
    Complete = 3
  };

  struct CtrlEntry
  {
    CtrlEntry()
    {
      flags.needsMeasure = 0;
      flags.sampled = 0;
    }

    // adc mux value to use for measurement
    unsigned int adcMUX = 0u;

    MeasurementMode mMode = MeasurementMode::Disabled;

    volatile TriggerMode tMode = TriggerMode::Immediate;

    struct Flags
    {
      uint8_t needsMeasure : 1;
      uint8_t sampled      : 1;// set to 1 when ADC is sampled. user can clear if desired
    };
    volatile Flags flags;

    // latest ADC measurement value
    volatile uint16_t value = 0u;

  };

  extern CtrlEntry ppsA;
  extern CtrlEntry ppsB;
  extern CtrlEntry tpsA;
  extern CtrlEntry tpsB;
  extern CtrlEntry driverFB;

  // running count of individual ADC conversions completed
  volatile extern uint16_t conversionCount;

  // running count of ADC schedule cycles completed
  volatile extern uint16_t conversionCycles;

  // @return
  // 1 if measurements were started
  // 0 if there are no enabled entries
  uint8_t
  start();

  void
  stop();

  uint8_t
  getSchedIdx();

  ADC_State
  getState();

}
