# octasonic-firmware

This is the official firmware for the [Octasonic HC-SR04 Breakout](https://www.tindie.com/products/andygrove73/octasonic-8-x-hc-sr04-ultrasonic-breakout-board/) that allows up to 8 HC-SR04 ultrasonic sensors to be monitored via SPI. 

# Prerequisites

Make sure you have an appropriate AVR toolchain installed.

I originally used the following instructions from 2015:

https://web.archive.org/web/20150617015707/http://maxembedded.com/2015/06/setting-up-avr-gcc-toolchain-on-linux-and-mac-os-x/

## Mac

Tested on Apple Silicon, Mac Ventura 13.5,  October 2023

- Install Homebrew
- `brew install avr-gcc`
- `brew install avrdude`

You will also need an AVR programmer to connect to the Octasonic board. 
I use [this one](https://www.sparkfun.com/products/9825). 

# Flashing Firmware

Connect the Octasonic board to the AVR Programmer and connect the AVR Programmer to a USB port on your computer and run the following commands.

```
cd firmware
make clean
make flash
make fuses
```


