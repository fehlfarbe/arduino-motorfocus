# arduino-motorfocus

Simple moonlite focuser protocol implementation for Arduino and stepper motors.

Tested with Arduino Nano and cheap [28BYJ-48 stepper motor with driver](https://arduino-info.wikispaces.com/SmallSteppers) and [INDI / Ekos](http://indilib.org). Works also with [Pololu A4988 Stepper Motor Driver](https://www.pololu.com/product/1182) (see wiring).

Updated 3D/CAD models for TS 65/420 Quadruplet are available at Thingiverse: https://www.thingiverse.com/thing:2063325
Motor focuser for Starlight Feather Touch 2" by Cover1987: https://www.thingiverse.com/thing:3593910

## Wiring

### Wiring for 28BYJ-48 stepper motor with ULN2003A breakout board:
![alt text](res/wiring.png)

### Wiring for A4988 driver
Change Line #13 in `arduino-motorfocus.ino` from `//#define USE_DRIVER` to `#define USE_DRIVER`
![alt text](res/wiring_driver.png)

### Wiring for in/out buttons
The internal pullups are active so you have to connect your buttons to GND.
![alt text](res/wiring_buttons.png)

### Wiring for temperature sensor DS18B20
You need a 4.7kΩ pullup resistor. If there is no sensor attached the temperatur will be zero.
![alt text](res/wiring_temperature.png)

## Arduino-motorfocus mounted on TS65/420 Quadruplet

![alt text](res/image01.jpg)

![alt text](res/image02.jpg)

