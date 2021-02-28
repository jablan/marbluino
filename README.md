# Marbluino
## Capture all the flags with the marble you control by tilting the game

A simple Arduino game uses MMA8451 accelerometer and Nokia 5110 (PCD8544 based) LCD display.

![Marbluino](/marbluino.jpg)

### Parts needed:

* A 3.3V microcontroller (current code works for Wemos D1 mini and Arduino Pro Mini, should be easy to adapt for others)
* PCD8544 based 84x48 pixel display. The game could easily be adapted to other displays and resolutions. PCD8544 is 3.3V based, so if you are using 5V board, you'll have to use resistors or IC to bring the voltage down. I was using 3.3V board.
* MMA8451 based accelerometer
* Buzzer and appropriate resistor

### Libraries used:

* [u8g2](https://github.com/olikraus/u8g2) - for the graphic
* [wire](https://www.arduino.cc/en/reference/wire) - i2c library for the accelerometer

### Fritzing sketch

![Fritzing Breadboard](/marbluino_bb.png)
