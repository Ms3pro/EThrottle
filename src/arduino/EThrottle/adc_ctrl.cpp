#include "adc_ctrl.h"

#include <Arduino.h>
#include <avr/interrupt.h>

namespace adc
{

  // declare external variables
  CtrlEntry ppsA;
  CtrlEntry ppsB;
  CtrlEntry tpsA;
  CtrlEntry tpsB;
  CtrlEntry driverFB;
  volatile uint16_t conversionCount;
  volatile uint16_t conversionCycles;
  volatile uint8_t schedIdx = 0;
  volatile ADC_State adcState = ADC_State::Stopped;

  // order in which we perform measurements
  const CtrlEntry *sched[] = {
    &ppsA,
    &ppsB,
    &tpsA,
    &tpsB,
    &driverFB
  };

  #define ARRAY_LEN(a) (sizeof(a) / sizeof(a[0]))

  // ADC prescaler options
  #define ADC_PS_2   0x1
  #define ADC_PS_4   0x2
  #define ADC_PS_8   0x3
  #define ADC_PS_16  0x4
  #define ADC_PS_32  0x5
  #define ADC_PS_64  0x6
  #define ADC_PS_128 0x7

  #define ADC_BASE_CFG ((1 << ADEN) | (1 << ADIE) | ADC_PS_128)

  // toggle a pin on different ADC events
  #define ENABLE_ADC_STROBES  0 // global enable
  #define STROBE_PIN A4
  #define STROBE_ON_ADC_START 0
  #define STROBE_ON_OC0A_ISR  1
  #define STROBE_ON_OC0B_ISR  1

  void
  startADC(
    const CtrlEntry *entry)
  {
    // setup ADC mux with Vcc as reference
    ADMUX = (1 << REFS0) | (entry->adcMUX & 0xF);

    switch (entry->tMode)
    {
      case TriggerMode::Immediate:
        adcState = ADC_State::Started;
        ADCSRB = 0x0;// trigger on start
        ADCSRA = ADC_BASE_CFG | (1 << ADSC);// start conversion
        break;
      case TriggerMode::Tmr0_MatchA:
      case TriggerMode::Tmr0_Ovrf:
      case TriggerMode::Tmr1_MatchB:
      case TriggerMode::Tmr1_Ovrf:
      case TriggerMode::Tmr1_CapEvt:
        // setup auto trigger source
        adcState = ADC_State::PendingTrigger;
        ADCSRB = (uint8_t)(entry->tMode);
        ADCSRA = ADC_BASE_CFG | (1 << ADATE);// enable conversion on auto trigger
        break;
      case TriggerMode::ISR_Tmr0_OCA:
        // will start conversion in the TIMER0_COMPA_vect ISR
        adcState = ADC_State::PendingTrigger;
        TIFR0 |= (1 << OCF0A);// clear interrupt flag
        TIMSK0 |= (1 << OCIE0A);
        break;
      case TriggerMode::ISR_Tmr0_OCB:
        // will start conversion in the TIMER0_COMPB_vect ISR
        adcState = ADC_State::PendingTrigger;
        TIFR0 |= (1 << OCF0B);// clear interrupt flag
        TIMSK0 |= (1 << OCIE0B);
        break;
    }

#if ENABLE_ADC_STROBES && STROBE_ON_ADC_START
    digitalWrite(STROBE_PIN, 1);
    digitalWrite(STROBE_PIN, 0);
#endif
  }

  // @return
  // 1 if a measurement was found and setup
  // 0 otherwise
  uint8_t
  findAndStartNext()
  {
    // find and setup next entry
    uint8_t origIdx = schedIdx;
    uint8_t doSetup = 0;
    uint8_t keepSearching = 1;
    while (keepSearching)
    {
      schedIdx++;
      if (schedIdx >= ARRAY_LEN(sched))
      {
        schedIdx = 0;
        conversionCycles++;
      }
      if (schedIdx == origIdx)
      {
        // went all the way around the schedule. stop looking
        keepSearching = 0;
      }

      switch (sched[schedIdx]->mMode)
      {
        case MeasurementMode::Disabled:
          continue;
          break;
        case MeasurementMode::Continuous:
          keepSearching = 0;
          doSetup = 1;
          break;
        case MeasurementMode::OneShot:
          if (sched[schedIdx]->flags.needsMeasure)
          {
            keepSearching = 0;
            doSetup = 1;
          }
          break;
      }
    }

    if (doSetup)
    {
      startADC(sched[schedIdx]);
    }
    else
    {
      adcState = ADC_State::Stopped;
    }
    return doSetup;
  }

  ISR(TIMER0_COMPA_vect)
  {
    TIMSK0 &= ~(1 << OCIE0A);// disable interrupt

    // start the ADC conversion (ADCMUX was setup for us in startADC())
    adcState = ADC_State::Started;
    ADCSRA = ADC_BASE_CFG | (1 << ADSC);

#if ENABLE_ADC_STROBES && STROBE_ON_OC0A_ISR
    digitalWrite(STROBE_PIN, 1);
    digitalWrite(STROBE_PIN, 0);
#endif
  }

  ISR(TIMER0_COMPB_vect)
  {
    TIMSK0 &= ~(1 << OCIE0B);// disable interrupt

    // start the ADC conversion (ADCMUX was setup for us in startADC())
    adcState = ADC_State::Started;
    ADCSRA = ADC_BASE_CFG | (1 << ADSC);

#if ENABLE_ADC_STROBES && STROBE_ON_OC0B_ISR
    digitalWrite(STROBE_PIN, 1);
    digitalWrite(STROBE_PIN, 0);
#endif
  }

  // ADC measurement complete interrupt
  ISR(ADC_vect)
  {
    adcState = ADC_State::Complete;
    conversionCount++;
    CtrlEntry *currEntry = sched[schedIdx];
    currEntry->value = ADC;
    currEntry->flags.needsMeasure = 0;
    currEntry->flags.sampled = 1;

    findAndStartNext();
  }

  uint8_t
  start()
  {
#if ENABLE_ADC_STROBES
    pinMode(STROBE_PIN, OUTPUT);
#endif

    // reset counters
    conversionCount = 0;
    conversionCycles = 0;

    // disable digital input on pins used for ADC
    DIDR0 = 0x0;
    for (uint8_t i=0; i<ARRAY_LEN(sched); i++)
    {
      if (sched[i]->adcMUX <= 5)
      {
        DIDR0 |= (1 << sched[i]->adcMUX);
      }
    }

    // start conversions
    const uint8_t setupOk = findAndStartNext();
    if (setupOk)
    {
      // wait for a full conversion cycle to complete
      while (conversionCycles == 0) {}
    }
    return setupOk;
  }

  void
  stop()
  {
    ADCSRA = 0x0;// terminate by disabling ADC
    adcState = ADC_State::Stopped;
  }

  uint8_t
  getSchedIdx()
  {
    return schedIdx;
  }

  ADC_State
  getState()
  {
    return adcState;
  }

}