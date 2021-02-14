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
#define MIN_DISTANCE 30 // avoid spawning flags too close to the ball
#define BADDIE_RATE 5 // spawn new baddie on every nth gathered flag

uint8_t max_x, max_y, points, timer = MAX_TIMER;
uint8_t baddies[5][2];
float x, y, dx = 0.0, dy = 0.0;
uint8_t bx, by;
uint16_t tonesFlag[][2] = {{698, 1}, {880, 1}, {1047, 1}, {0, 0}};
uint16_t tonesLevel[][2] = {{1047, 1}, {698, 1}, {880, 1}, {1047, 1}, {0, 0}};
uint8_t melodyIndex;
uint16_t (*melody)[2];

void drawBoard(void) {
  static char buf[12];
  u8g2.firstPage();
  do {
    // draw marble
    u8g2.drawDisc(x, y, BALLSIZE/2);
    // draw flag
    u8g2.drawTriangle(bx, by-3, bx-3, by+2, bx+3, by+2);
    // draw baddies
    for(int i=0; i <= baddiesCount(); i++) {
      u8g2.drawFrame(baddies[i][0]-2, baddies[i][1]-2, 4, 4);
    }
    // write points and time
    itoa(points, buf, 10);
    u8g2.drawStr(0, 5, buf);
    itoa(timer/10, buf, 10);
    uint8_t width = u8g2.getStrWidth(buf);
    u8g2.drawStr(max_x-width, 5, buf);
  } while(u8g2.nextPage());
}

void placeRandomly(uint8_t *cx, uint8_t *cy) {
  do {
    *cx = random(max_x - 2*BALLSIZE) + BALLSIZE;
    *cy = random(max_y - 2*BALLSIZE) + BALLSIZE;
  } while (abs(*cx-x) + abs(*cy-y) < MIN_DISTANCE);
}

void playSound(void) {
  if (melody) {
    uint8_t totalCount = 0;
    for (uint8_t i = 0; 1; i++) {
      uint16_t freq = melody[i][0];
      uint16_t dur = melody[i][1];
      if (melodyIndex == totalCount) {
        if (dur == 0) {
          noTone(BUZZER_PIN);
          melody = NULL;
          melodyIndex = 0;
        } else {
          tone(BUZZER_PIN, freq);
        }
      }
      totalCount += dur;
      if (totalCount > melodyIndex)
        break;
    }
    melodyIndex++;
  }
}

int baddiesCount(void) {
  return points/BADDIE_RATE;
}

bool isCollided(uint8_t cx, uint8_t cy) {
  return abs(x-cx) < 3 && abs(y-cy) < 3;
}

void melodySad(void) {
}

void melodyFlag(void) {
  melody = tonesFlag;
  melodyIndex = 0;
}

void melodyLevel(void) {
  melody = tonesLevel;
  melodyIndex = 0;
}

void gameOver(void) {
  points = 0;
  timer = MAX_TIMER;
  x = max_x / 2;
  y = max_y / 2;
  placeRandomly(&bx, &by);
  melodySad();
  delay(1000);
}

void checkCollision(void) {
  for(int i=0; i <= baddiesCount(); i++) {
    if (isCollided(baddies[i][0], baddies[i][1])) {
      return gameOver();
    }
  }
  if (isCollided(bx, by)) {
    placeRandomly(&bx, &by);
    points++;
    timer = MAX_TIMER;
    if (points % BADDIE_RATE == 0) {
      // spawn new baddie
      placeRandomly(&(baddies[points / BADDIE_RATE][0]), &(baddies[points / BADDIE_RATE][1]));
      melodyLevel();
    } else {
      melodyFlag();
    }
  }
}

void updateMovement(void) {
  x += dx;
  y += dy;
  float xyz_g[3];
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
  max_y = u8g2.getDisplayHeight();
  x = max_x / 2;
  y = max_y / 2;
  placeRandomly(&bx, &by);
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
