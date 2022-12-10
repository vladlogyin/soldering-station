# cortexm-dev
This is a template repository meant to aid in the writing of embedded arm projects
## What's included
The included main.cpp is a blinky "sketch" intended to run on a blackpill dev board (LED on pin PB12)
## Features
* uses [libopencm3](https://github.com/libopencm3/libopencm3) as an embedded library
## How to use
* Make sure your toolchain is set up by running `make checktoolchain`
* Just run `make flash` and everything should automatically build and the resulting binary should be flashed to your target
* If you want to use gdb to debug your target, just run `make debug`
## How to customize
1. Create a linker script for your target MCU
2. Create an openOCD flash script
3. Create an openOCD debug script
4. Tinker with the MCU specific settings inside of the Makefile

## Currently supported hardware
Basically, all MCUs supported by [libopencm3](https://github.com/libopencm3/libopencm3):

- ST STM32 F0xx/F1xx/F2xx/F30x/F37x/F4xx/F7xx/H7xx series
- ST STM32 G0xx G4xx L0xx L1xx L4xx series
- Atmel SAM3A/3N/3S/3U/3X series, as well as SAMDxx and friends
- NXP LPC1311/13/17/42/43
- Stellaris LM3S series (discontinued, without replacement)
- TI (Tiva) LM4F series (continuing as TM4F, pin and peripheral compatible)
- EFM32 Gecko series (only core support)
- Freescale Vybrid VF6xx
- Qorvo (formerly ActiveSemi) PAC55XX
- Synwit SWM050
### Sensors
- BME280
- LM75
### Displays
- ILI9341 (no DMA yet)
- MAX7219
### Misc.
- RC522 NFC reader

## TODO
- [ ] Add comments to code and integrate doxygen
- [ ] Add a USB Mass Storage example
- [X] Add a USB CDC ACM example
- [ ] Add a CAN example
