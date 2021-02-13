#include <Arduino.h>
#include <U8g2lib.h>
#include "i2c.h"
#include "i2c_MMA8451.h"

MMA8451 mma8451;
U8G2_PCD8544_84X48_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);

#define BUZZER_PIN 6

#define BALLSIZE 4
#define ACC_FACTOR 0.5
#define BOUNCE_FACTOR -0.5
#define DELAY 50
#define MAX_TIMER 10*1000/DELAY

uint8_t max_x, max_y, points, timer = MAX_TIMER, toneIndex = 0;
float x, y, dx = 0.0, dy = 0.0;
float bx, by;

void drawBoard(void) {
  static char buf[12];
  u8g2.firstPage();
  do {
    u8g2.drawDisc(x, y, BALLSIZE/2);
    u8g2.drawTriangle(bx, by-3, bx-3, by+2, bx+3, by+2);
    itoa(points, buf, 10);
    u8g2.drawStr( 0, 5, buf);
    itoa(timer/10, buf, 10);
    uint8_t width = u8g2.getStrWidth(buf);
    u8g2.drawStr(max_x-width, 5, buf);
  } while(u8g2.nextPage());
}

void placeBeacon(void) {
  bx = random(max_x - 2*BALLSIZE) + BALLSIZE;
  by = random(max_y - 2*BALLSIZE) + BALLSIZE;
}

void playSound(void) {
  if (toneIndex == 3) {
    tone(BUZZER_PIN, 698, DELAY);
    toneIndex--;
  } else if (toneIndex == 2) {
    tone(BUZZER_PIN, 880, DELAY);
    toneIndex--;
  } else if (toneIndex == 1) {
    tone(BUZZER_PIN, 1047, DELAY);
    toneIndex--;
  }
}

void checkCollision(void) {
  if (abs(x-bx) < 3 && abs(y-by) < 3) {
    toneIndex = 3;
    placeBeacon();
    points++;
    timer = MAX_TIMER;
  }
}

void updateMovement(void) {
  x += dx;
  y += dy;
  static float xyz_g[3];
  mma8451.getMeasurement(xyz_g);
  dx += ACC_FACTOR * (-xyz_g[0]);
  dy += ACC_FACTOR * xyz_g[1];
}

void bounce(void) {
  if ((dx > 0 && x >= max_x-BALLSIZE) || (dx < 0 && x <= BALLSIZE)) {
    dx = BOUNCE_FACTOR * dx;
  }
  if ((dy > 0 && y >= max_y-BALLSIZE) || (dy < 0 && y <= BALLSIZE)) {
    dy = BOUNCE_FACTOR * dy;
  }
}

void setup(void) {
  randomSeed(analogRead(0));
//  Serial.begin(115200);
  mma8451.initialize();
  u8g2.begin();
  u8g2.setFont(u8g2_font_baby_tn);

  max_x = u8g2.getDisplayWidth();
  x = max_x / 2;
  max_y = u8g2.getDisplayHeight();
  y = max_y / 2;
  placeBeacon();
}

void loop(void) {
  bounce();
  checkCollision();
  drawBoard();
  updateMovement();

  playSound();
  delay(DELAY);
  if (timer > 0)
    timer--;
}
