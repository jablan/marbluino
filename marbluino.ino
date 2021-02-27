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
#include <avr/sleep.h>

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

uint8_t max_x, max_y, points, timer = MAX_TIMER;
uint8_t baddies[5][2];
float x, y, dx = 0.0, dy = 0.0;
uint8_t bx, by;
uint16_t tonesFlag[][2] = {{698, 1}, {880, 1}, {1047, 1}, {0, 0}};
uint16_t tonesLevel[][2] = {{1047, 1}, {988, 1}, {1047, 1}, {988, 1}, {1047, 1}, {0, 0}};
uint16_t tonesSad[][2] = {{262, 1}, {247, 1}, {233, 1}, {220, 3}, {0, 0}};

#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262

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

void placeRandomly(uint8_t *cx, uint8_t *cy) {
  do {
    *cx = random(max_x - 2*BALLSIZE) + BALLSIZE;
    *cy = random(max_y - 2*BALLSIZE) + BALLSIZE;
  // ensure not spawning too close to the ball
  } while (abs(*cx-x) + abs(*cy-y) < MIN_DISTANCE);
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

bool isCollided(uint8_t cx, uint8_t cy) {
  return abs(x-cx) < 3 && abs(y-cy) < 3;
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
  x = max_x / 2;
  y = max_y / 2;
  placeRandomly(&bx, &by);
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
  getOrientation(xyz_g);

  Serial.print("X:\t"); Serial.print(xyz_g[0]); 
  Serial.print("\tY:\t"); Serial.print(xyz_g[1]); 
  Serial.print("\tZ:\t"); Serial.print(xyz_g[2]);
  Serial.println();

  dx += ACC_FACTOR * (-xyz_g[0]);
  dy += ACC_FACTOR * xyz_g[1];
}

// bounce off walls with a diminishing factor
void bounce(void) {
  if ((dx > 0 && x >= max_x-BALLSIZE) || (dx < 0 && x <= BALLSIZE)) {
    dx = BOUNCE_FACTOR * dx;
  }
  if ((dy > 0 && y >= max_y-BALLSIZE) || (dy < 0 && y <= BALLSIZE)) {
    dy = BOUNCE_FACTOR * dy;
  }
}

void goToSleep() {
  showPopup("SLEEPING...", "shake to wake");
  mmaSetupMotionDetection();
#ifdef ESP8266
  ESP.deepSleep(0);
#endif
#ifdef __AVR__
set_sleep_mode(SLEEP_MODE_PWR_DOWN);
sleep_mode();
#endif
}

void setup(void) {
  randomSeed(analogRead(0));
  Serial.begin(115200);
#ifdef ESP8266
  WiFi.mode(WIFI_OFF);
#endif
  
  setupMMA();
 
  u8g2.begin();
  u8g2.setFont(u8g2_font_baby_tf);
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
