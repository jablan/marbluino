# Marbluino
## Capture all the flags with the marble you control by tilting the game

A simple Arduino game uses MMA845x accelerometer and Nokia 5110 (PCD8544 based) LCD display.

![Marbluino](/marbluino.jpg)

### Parts needed:

* A 3.3V microcontroller (current code works for Wemos D1 mini and Arduino Pro Mini, should be easy to adapt for others)
* PCD8544 based 84x48 pixel display. The game could easily be adapted to other displays and resolutions. PCD8544 is 3.3V based, so if you are using 5V board, you'll have to use resistors or IC to bring the voltage down. I was using 3.3V board.
* MMA845x based accelerometer
* Buzzer and appropriate resistor

### Libraries used:

* [u8g2](https://github.com/olikraus/u8g2) - for the graphic
* [ArduinoAccelerometerDriver](https://github.com/dalmirdasilva/ArduinoAccelerometerDriver) - library for several different accelerometers, includes code for triggering interrupts on move, pulse, freefall etc.

### Power saving

If the time runs out and the user does not have any points collects, the game assumes the user is not playing anymore and goes to sleep. However, the screen remains on, showing "shake to wake" message. The display consumes very low amount of energy itself.

The user wakes the game by shaking it - it works using the interrupt feature of MMA845x, which fires an interrupt when it detects a certain motion pattern. The interrupt pin is connected to reset pin of the board, it resets the game and game starts. This way, the consumption is reduced from ~70mA (while the game is running) to ~0.3mA (deep sleep on ESP8266).

### Fritzing sketches

#### Arduino Pro Mini
![Breadboard Arduino Pro Mini](/marbluino_bb.png)

#### Wemos D1 mini
![Breadboard Wemos D1 mini](/marbluino_8266_bb.png)
