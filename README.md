# ArduinoNTCThermostat
This project is an Arduino based Thermostat for universal use. I put it together rather quick&dirty since I needed a simple thermostat, and I could not find one that fit my needs...
The project contains the Arduino code and the KiCad files for the board.

I ended up with a rather generic setup, that may be of use to others. The thermostat is intended to be used standalone in 12V or 24 V environments. It's settings for cooling or heating mode, for the target temperature and the hysteresis can be set without recompilation using a bridge (heating/cooling) and DIP switches (target temperature and hysteresis). A relay is fitted that can switch up to 250V and 10A.
A SSD1306 compatible OLED display can be fitted as an option to display detailed information.

## Features:
- Temperature measurement is done via a 10K NTC sensor
- Powered using an efficient DCDC converter (Traco TSR 1-2450E pin compatible to 7805) for operation in range of 7-36 V DC. The thermostat can easily operate in 12 or 24 V installations.
- Set for heating or cooling operation with a bridge
- Set target temperature fron 0 to 254 °C in 2°C steps via DIP switches
- Set hysteresis from 0 to 14 °C in 2°C steps via DIP switches
- 250V 10A relay. PCB design supports up to 10 A
- 2 LEDs: power indicator and switch indicator
- Optional OLED display via I2C

## PCB
The PCB has been designed in Kicad. Kicad files are included in this repository, also the Gerber files that can directly be used to order the PCBs

![PCB](https://github.com/88markus88/ArduinoNTCThermostat/blob/main/Pictures/ArduinoNTCThermostat%20V0.2%20PCB.jpg)
![PCB](https://github.com/88markus88/ArduinoNTCThermostat/blob/main/Pictures/ArduinoNTCThermostat%20V0.2%203D-1.jpg)
![PCB](https://github.com/88markus88/ArduinoNTCThermostat/blob/main/Pictures/ArduinoNTCThermostat%20V0.2%203D-2.jpg)

## Parts List
- PCB: Gerber files are attached
- Arduino Pro Micro (with socket, if desired)
- Transistor
- Electrolyte Capacitor 47 uF / 25V
- Tantal Capacitor 100 nF
- Opto Coupler LTV 817
- 2 Diodes 1N4001 or 1N4004
- 2 LEDs
- Power via DCDC  Traco TSR 1-2450E, alternatively 7850
- 10x DIP Switch
- Screw Terminals 5.08 mm pitch: 2x 2, 1x 3 connections
- Relay Songle SRD-05VDC-SL-C
- Resistors: 200, 470 and 2 x 10K Ohm
