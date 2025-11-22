# Megasquirt Drive By Wire
This repo contains source code and PCB design files for a Megasquirt Extended CAN device that controls a drive by wire throttle body.

![block-diagram](/doc/block_diagram.svg)

## 🚧 Project Status Disclaimer

**USE AT YOUR OWN RISK!!!**  
This project is a **work in progress** and is **not yet fully validated**.  
I have **not installed or used this board in my own vehicle**, so any real-world behavior has **not** been tested. All testing so far has been limited to a test bench.

Please also read the **[Safety & Liability Disclaimer](#%EF%B8%8F-safety--liability-disclaimer)**.

## ✨ Current Features
- Tunable from Tuner Studio (via Megasquirt CAN passthrough protocol)
- Data logging support
- Built-in safety features:
  - Dual-sensor redundancy (compares PPS & TPS sensor pairs)
  - Motor driver over-current protection
  - Motor driver over-temperature protection
- CPU watchdog support
  - Enable via `WATCHDOG_SUPPORT` in [config.h](/src/arduino/EThrottle/config.h)
  - Requires the [mcp-can-boot](https://github.com/crycode-de/mcp-can-boot) bootloader, as the default Arduino bootloader does not correctly support watchdog resets

## 🛠️ Future Features
- **v2 PCB** with improved power supply protection (see issue #44)
  - Temporary fixes include added filter capactors on 12v and 5v rails and replacing the Arduino Nano's 5v regulator with a 7805. Without these modifications, the original regulator could fail during aggresive throttle movement, resulting in frying the MCU and CAN transceiver
- Cruise control functionality
- Support throttle "blip" requests (e.g. for automatic/sequential transmissions)

## Building The Arduino Project
This repo uses **git submodules** for library dependencies. Arduino IDE **1.6+** is required.

1. Clone the repo

2. Update the submodules
`git submodule update --init --recursive`

3. In Arduino IDE go to `File > Preference` and set the Arduino IDE sketchbook location to:
`<path_to_this_repo>/src/arduino/`

4. Open the project through: `File > Sketchbook > EThrottle`

All required libraries will be available under `src/arduino/libraries/`.

## Printed Circuit Board
The project files for the PCB are located in [/src/pcb/](/src/pcb/). They can be opened with [EasyEDA Pro](https://easyeda.com/). __Note that they will not open with EasyEDA Standard version__ (might convert them to std version later).

Here's a render of the printed circuit board (PCB).

![pcb_render](/doc/board_layout/pcb_v1.0.png)

The board is designed to be installed into the prototype area of the Megasquirt DIYPNP via a 20pin header connection.

![pcb_installed](/doc/board_layout/pcb_v1.0_installed.jpg)

## ⚠️ Safety & Liability Disclaimer

**This project implements a drive-by-wire throttle control system.  
It is a safety-critical automotive component.**

Use of this code, hardware design, or any related materials is **entirely at your own risk**.

- This project is provided **“as is”**, with **no warranties** of any kind.  
- No guarantee is made regarding safety, correctness, or fitness for any purpose.  
- **I am not liable** for any damages, malfunctions, failures, injuries, or losses that result from using, modifying, or relying on this project.

**Do not use this in any real vehicle** without proper engineering validation, redundancy, testing, and compliance with all relevant safety standards and regulations.