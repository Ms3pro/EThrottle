#ifndef CONFIG_H_
#define CONFIG_H_

//---------------------------------------------------------------------
// I/O Signals
//---------------------------------------------------------------------
// analog inputs
               // A0 TPS_A
               // A1 TPS_B
               // A2 PPS_A
               // A3 PPS_B
#define CTRL_BUTN A4
               // A5 DRIVER_FB
// digital I/O pins
#define WSPEED_INT 2
#define CAN_INT    3
#define DRIVER_DIS 4
#define DRIVER_P   5// pin needs PWM support (PD5 OC0B)
#define DRIVER_N   6// pin needs PWM support (PD6 OC0A)
#define DRIVER_FS  7
#define CLUTCH_SW  8
#define BRAKE_SW   9
#define CAN_CS     10
                // 11 CAN MOSI
                // 12 CAN MISO
                // 13 CAN SCK

//---------------------------------------------------------------------
// CAN Bus Configuration
//---------------------------------------------------------------------
#define CAN_ID  5
#define CAN_MSG_BUFFER_SIZE 8

//---------------------------------------------------------------------
// mcp-can-boot Bootloader Options
//---------------------------------------------------------------------
// define this if you have the mcp-can-boot bootloader installed
//  * allows for remote reflashing via CAN
//  * watchdog reset support (regular Arduino bootloader doesn't support)
#undef MCP_CAN_BOOT_BL

// auto-enable WATCHDOG_SUPPORT if it's not explicitly set and we
// have support for the mcp-can-boot bootloader.
#define WATCHDOG_SUPPORT 0
#if defined(MCP_CAN_BOOT_BL)
#define WATCHDOG_SUPPORT 1
#endif

//---------------------------------------------------------------------
// Throttle.h Class Options
//---------------------------------------------------------------------
#define PID_SAMPLE_RATE_MS 10 // 10ms

// define this if the motor driver is an h-bridge variant. this
// enables us to reverse the throttle motor polarity to close
// the blade faster than the return spring would allow.
// if unset, then the code will rely on the return spring to
// bring the throttle blade closed (slow but still functional)
#define SUPPORT_H_BRIDGE

#endif