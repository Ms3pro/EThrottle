#include <avr/sfr_defs.h>
#include <avr/wdt.h>

#include <EndianUtils.h>
#include <logging_impl_lite.h>

#include "adc_ctrl.h"
#include "can.h"
#include "config.h"
#include "ecu_vars.h"
#include "health_monitor.h"
#include "test_controller.h"
#include "Throttle.h"

ThrottleOutVars_T *throttleVars = &outPC.tVars;

Throttle throttle(
  DRIVER_P,DRIVER_N,
  DRIVER_DIS,
  DRIVER_FS);

HealthMonitor healthMonitor(&canDev, &throttle);

TestController testController;

// setup watchdog
void wdtInit() {
  cli();// disable interrupts
  wdt_reset();// reset watchdog
  wdt_enable(WDTO_15MS);// start watchdog timer with 15ms timeout
  sei();
}

const char *
showResetCause(
  const uint8_t mcusr)
{
  INFO("reset cause...");
  if (mcusr & (1 << 0))
  {
    INFO("Power-ON");
  }
  if (mcusr & (1 << 1))
  {
    INFO("External");
  }
  if (mcusr & (1 << 2))
  {
    INFO("Brown-Out");
  }
  if (mcusr & (1 << 3))
  {
    INFO("Watchdog");
  }
}

void setup() {
#if defined(MCP_CAN_BOOT_BL)
  // retrieves the MCU reset cause (MCUSR register) when using the
  // mcp-can-boot bootloader (https://github.com/crycode-de/mcp-can-boot).
  uint8_t mcusr;
  __asm__ __volatile__ ( "mov %0, r2 \n" : "=r" (mcusr) : );
#endif

#if WATCHDOG_SUPPORT == 1
  wdtInit();// start watchdog
#endif

  setupLogging(115200);
  INFO("WATCHDOG: %s", (WATCHDOG_SUPPORT ? "ON" : "OFF"));
#ifdef MCP_CAN_BOOT_BL
  outPC.mcusr.word = mcusr;
  showResetCause(mcusr);
#else
  outPC.mcusr.word = 0;// arduino bootloader doesn't preserve MCUSR contents
#endif

  throttle.init(PID_SAMPLE_RATE_MS, throttleVars);
  loadFlashPage1ToThrottle(throttle);
  canSetup();
  adc::start();
}

void updateStatus() {
  outPC.status0.bits.msqRT_BCastListenFault = ! healthMonitor.getStatus().bits.ecuRtDataOkay;

  // update ADC status
  outPC.adcStatus.schedIdx = adc::getSchedIdx();
  outPC.adcStatus.state = static_cast<uint8_t>(adc::getState());
  outPC.adcStatus.convCycles = adc::conversionCycles;
  outPC.adcStatus.adcsra = ADCSRA;
}

void doTestMode() {
}

void throttleLoop() {
  if (outPC.status1.bits.testModeEnabled)
  {
    testController.run();
  }
  else
  {
    throttle.setIdleAddFactor(
      healthMonitor.getStatus().bits.ecuRtDataOkay ? ecu::idleDuty : 0u);
  }
  throttle.run();
}

void loop() {
  wdt_reset();// throw watchdog a bone
  const auto loopStartTimeUs = micros();

  healthMonitor.run();
  updateStatus();
  canLoop();
  throttleLoop();

  // update loop time register
  const auto loopTimeUs = static_cast<uint16_t>(micros() - loopStartTimeUs);
  EndianUtils::setBE(outPC.loopTimeUs, loopTimeUs);
}
