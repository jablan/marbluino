# Marbluino
## Capture all the flags with the marble you control by tilting the game

A simple Arduino game uses MMA8451 accelerometer and Nokia 5110 (PCD8544 based) LCD display.

### Parts needed:

* Arduino (pretty much any should work, I was using 3.3V Mini Pro with ATmega168)
* PCD8544 based 84x48 pixel display. The game could easily be adapted to other displays and resolutions. PCD8544 is 3.3V based, so if you are using 5V board, you'll have to use resistors or IC to bring the voltage down. I was using 3.3V board.
* MMA8451 based accelerometer
* Buzzer and appropriate resistor

### Libraries used:

* [u8g2](https://github.com/olikraus/u8g2) - for the graphic
* [I2C-Sensor-Lib](https://github.com/orgua/iLib) - for the accelerometer

### Fritzing sketch

![Fritzing Breadboard](/marbluino_bb.png)