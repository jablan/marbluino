#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>

#define BUZZER_PIN D8
#define DISPLAY_CS_PIN D3
#define DISPLAY_DC_PIN D0
#define DISPLAY_RS_PIN D4
#endif

#ifdef __AVR__
#include "LowPower.h"

#define BUZZER_PIN 6
#define DISPLAY_CS_PIN 10
#define DISPLAY_DC_PIN 9
#define DISPLAY_RS_PIN 8
#endif

#define BALLSIZE 4
#define ACC_FACTOR 0.5
#define BOUNCE_FACTOR -0.5
#define DELAY 50
#define MAX_TIMER 10*1000/DELAY
#define MIN_DISTANCE 30 // avoid spawning flags too close to the ball
#define BADDIE_RATE 5 // spawn new baddie on every nth gathered flag

U8G2_PCD8544_84X48_F_4W_HW_SPI u8g2(U8G2_R0, DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RS_PIN);

struct fpoint {
  float x;
  float y;
};

struct upoint {
  uint8_t x;
  uint8_t y;
};

uint8_t max_x, max_y, points, timer = MAX_TIMER;
struct fpoint ball, speed = {0.0, 0.0};
struct upoint flag, baddies[5];
uint16_t tonesFlag[][2] = {{698, 1}, {880, 1}, {1047, 1}, {0, 0}};
uint16_t tonesLevel[][2] = {{1047, 1}, {988, 1}, {1047, 1}, {988, 1}, {1047, 1}, {0, 0}};
uint16_t tonesSad[][2] = {{262, 1}, {247, 1}, {233, 1}, {220, 3}, {0, 0}};

uint8_t melodyIndex;
uint16_t (*currentMelody)[2];

// MMA8452Q I2C address is 0x1C(28)
#define MMA_ADDR 0x1C

void mmaRegWrite(byte reg, byte value) {
  Wire.beginTransmission(MMA_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void mmaSetStandbyMode() {
  mmaRegWrite(0x2A, 0x18); //Set the device in 100 Hz ODR, Standby  
}

void mmaSetActiveMode() {
  mmaRegWrite(0x2A, 0x19);  
}

// Causes interrupt when shaken
void mmaSetupMotionDetection() {
  // https://www.nxp.com/docs/en/application-note/AN4070.pdf
  mmaSetStandbyMode();
  mmaRegWrite(0x15, 0x78);
  mmaRegWrite(0x17, 0x1a);
  mmaRegWrite(0x18, 0x10);
  // enable interrupt
  mmaRegWrite(0x2D, 0x04);
  mmaRegWrite(0x2E, 0x04);
  mmaSetActiveMode();
}

void mmaDisableInterrupt() {
  mmaRegWrite(0x2d, 0x00);
}

void setupMMA()
{
  // Initialise I2C communication as MASTER
  Wire.begin();

  mmaSetStandbyMode();
  mmaDisableInterrupt();
  mmaRegWrite(0x0e, 0x00); // set range to +/- 2G
  mmaSetActiveMode();
}

void getOrientation(float xyz_g[3]) {
  unsigned int data[7];

  // Request 7 bytes of data
  Wire.requestFrom(MMA_ADDR, 7);
 
  // Read 7 bytes of data
  // status, xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
  if(Wire.available() == 7) 
  {
    for (int i = 0; i<6; i++) {
      data[i] = Wire.read();
    }
  }

  // Convert the data to 12-bits
  int iAccl[3];
  for (int i = 0; i < 3; i++) {
    iAccl[i] = ((data[i*2+1] << 8) | data[i*2+2]) >> 4;
    if (iAccl[i] > 2047)
    {
      iAccl[i] -= 4096;
    }
    xyz_g[i] = (float)iAccl[i] / 1024;
  }
}

void drawBoard(void) {
  static char buf[12];
  u8g2.clearBuffer();
  // draw marble
  u8g2.drawDisc(ball.x, ball.y, BALLSIZE/2);
  // draw flag
  u8g2.drawTriangle(flag.x, flag.y-3, flag.x-3, flag.y+2, flag.x+3, flag.y+2);
  // draw baddies
  for(int i=0; i <= baddiesCount(); i++) {
    u8g2.drawFrame(baddies[i].x-2, baddies[i].y-2, 4, 4);
  }
  // write points and time
  itoa(points, buf, 10);
  u8g2.drawStr(0, 5, buf);
  itoa(timer/10, buf, 10);
  uint8_t width = u8g2.getStrWidth(buf);
  u8g2.drawStr(max_x-width, 5, buf);
  u8g2.sendBuffer();
}

void showPopup(char *line_1, char *line_2) {
  u8g2.clearBuffer();
  u8g2.drawRFrame(0, 0, max_x, max_y, 7);
  uint8_t width = u8g2.getStrWidth(line_1);
  u8g2.drawStr((max_x-width)/2, max_y/2 - 2, line_1);
  width = u8g2.getStrWidth(line_2);
  u8g2.drawStr((max_x-width)/2, max_y/2 + 8, line_2);
  u8g2.sendBuffer();
}

void placeRandomly(struct upoint *point) {
  do {
    (*point).x = random(max_x - 2*BALLSIZE) + BALLSIZE;
    (*point).y = random(max_y - 2*BALLSIZE) + BALLSIZE;
  // ensure not spawning too close to the ball
  } while (abs((*point).x-ball.x) + abs((*point).y-ball.y) < MIN_DISTANCE);
}

// used to play the melody asynchronously while the user is playing
void playSound(void) {
  if (currentMelody) {
    uint8_t totalCount = 0;
    for (uint8_t i = 0; 1; i++) {
      uint16_t freq = currentMelody[i][0];
      uint16_t dur = currentMelody[i][1];
      if (melodyIndex == totalCount) {
        if (dur == 0) {
          noTone(BUZZER_PIN);
          currentMelody = NULL;
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

bool isCollided(struct upoint point) {
  return abs(ball.x-point.x) < 3 && abs(ball.y-point.y) < 3;
}

void melodySad(void) {
  // this is played synchronously
  for (uint8_t i = 0; tonesSad[i][1] > 0; i++) {
    tone(BUZZER_PIN, tonesSad[i][0], tonesSad[i][1]*300);
    delay(tonesSad[i][1] * 300 + 50);
  }
}

void melodyFlag(void) {
  currentMelody = tonesFlag;
  melodyIndex = 0;
}

void melodyLevel(void) {
  currentMelody = tonesLevel;
  melodyIndex = 0;
}

void gameOver(void) {
  char msg[50];
  sprintf(msg, "score: %d", points);
  showPopup("GAME OVER", msg);
  melodySad();
  points = 0;
  timer = MAX_TIMER;
  ball.x = max_x / 2;
  ball.y = max_y / 2;
  placeRandomly(&flag);
}

void checkCollision(void) {
  for(int i=0; i <= baddiesCount(); i++) {
    if (isCollided(baddies[i])) {
      return gameOver();
    }
  }
  if (isCollided(flag)) {
    placeRandomly(&flag);
    points++;
    timer = MAX_TIMER;
    if (points % BADDIE_RATE == 0) {
      // spawn new baddie
      placeRandomly(&(baddies[points / BADDIE_RATE]));
      melodyLevel();
    } else {
      melodyFlag();
    }
  }
}

void updateMovement(void) {
  ball.x += speed.x;
  ball.y += speed.y;
  float xyz_g[3];
  getOrientation(xyz_g);

#ifdef DEBUG
  Serial.print("X:\t"); Serial.print(xyz_g[0]); 
  Serial.print("\tY:\t"); Serial.print(xyz_g[1]); 
  Serial.print("\tZ:\t"); Serial.print(xyz_g[2]);
  Serial.println();
#endif

  speed.x += ACC_FACTOR * (-xyz_g[0]);
  speed.y += ACC_FACTOR * xyz_g[1];
}

// bounce off walls with a diminishing factor
void bounce(void) {
  if ((speed.x > 0 && ball.x >= max_x-BALLSIZE) || (speed.x < 0 && ball.x <= BALLSIZE)) {
    speed.x = BOUNCE_FACTOR * speed.x;
  }
  if ((speed.y > 0 && ball.y >= max_y-BALLSIZE) || (speed.y < 0 && ball.y <= BALLSIZE)) {
    speed.y = BOUNCE_FACTOR * speed.y;
  }
}

void goToSleep() {
  showPopup("SLEEPING...", "shake to wake");
  mmaSetupMotionDetection();
#ifdef ESP8266
  ESP.deepSleep(0);
#endif
#ifdef __AVR__
LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
#endif
}

void setup(void) {
  randomSeed(analogRead(0));
#ifdef DEBUG
  Serial.begin(115200);
#endif
#ifdef ESP8266
  WiFi.mode(WIFI_OFF);
#endif
  
  setupMMA();
 
  u8g2.begin();
  u8g2.setFont(u8g2_font_baby_tf);
  max_x = u8g2.getDisplayWidth();
  max_y = u8g2.getDisplayHeight();
  ball.x = max_x / 2;
  ball.y = max_y / 2;
  placeRandomly(&flag);
}

void loop(void) {
  bounce();
  checkCollision();
  drawBoard();
  updateMovement();

  playSound();
  delay(DELAY);
  if (timer > 0) {
    timer--;
  } else {
    if (points == 0) {
      goToSleep();    
    } else {
      gameOver();
    }
  }
}
