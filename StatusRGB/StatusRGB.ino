#include <Adafruit_NeoPixel.h>

#define DIGITAL_D0 8
#define DIGITAL_D1 9
#define DIGITAL_D2 10
#define DIGITAL_D3 11
#define DIGITAL_D4 12

#define OUTPUT_D0 3
#define OUTPUT_D1 4
#define OUTPUT_D2 5
#define OUTPUT_D3 6
#define OUTPUT_D4 7

#define LEDSTRIP_FRONT A4
#define LEDSTRIP_SIDES A5

#define NUMPIXELS_FRONT 10  // number of neopixels in strip
#define NUMPIXELS_SIDES 10  // number of neopixels in strip

#define DELAY_TIME 200
#define INTENSITY 255

#define IDLE_CYCLE 100
#define IDLE_BLOCK 8

Adafruit_NeoPixel pixelsFront(NUMPIXELS_FRONT, LEDSTRIP_FRONT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixelsSides(NUMPIXELS_SIDES, LEDSTRIP_SIDES, NEO_GRB + NEO_KHZ800);

uint32_t lowBlue = pixelsFront.Color(0, 0, INTENSITY / 3);
uint32_t highBlue = pixelsFront.Color(0, 0, INTENSITY);
uint32_t lowGold = pixelsFront.Color(INTENSITY / 3, INTENSITY / 6, 0);
uint32_t highGold = pixelsFront.Color(INTENSITY, INTENSITY / 2, 0);
uint32_t fullRed = pixelsFront.Color(255, 0, 0);
uint32_t fullGreen = pixelsFront.Color(0, 255, 0);  
uint32_t fullBlue = pixelsFront.Color(0, 0, 255);
uint32_t purple = pixelsFront.Color(128, 0, 128);
uint32_t cyan = pixelsFront.Color(0, 255, 255);

int mode = 0;
int timer = 0;

unsigned long myTime;

void setup() {
  Serial.begin(250000);

  pinMode(DIGITAL_D0, INPUT_PULLUP);
  pinMode(DIGITAL_D1, INPUT_PULLUP);
  pinMode(DIGITAL_D2, INPUT_PULLUP);
  pinMode(DIGITAL_D3, INPUT_PULLUP);
  pinMode(DIGITAL_D4, INPUT_PULLUP);

  pinMode(OUTPUT_D0, OUTPUT);
  pinMode(OUTPUT_D1, OUTPUT);
  pinMode(OUTPUT_D2, OUTPUT);
  pinMode(OUTPUT_D3, OUTPUT);
  pinMode(OUTPUT_D4, OUTPUT);

  pixelsFront.begin();
}

void setColor(bool red, bool green, bool blue, Adafruit_NeoPixel *pixels, int size) {
  pixels->clear();
  for (int i = 0; i < size; i++) {
    pixels->setPixelColor(i, pixels->Color(INTENSITY * (int)red, INTENSITY * (int)green * .50, INTENSITY * (int)blue));
  }
  pixels->show();
}

void setColorInt(int red, int green, int blue, Adafruit_NeoPixel *pixels, int size) {
  pixels->clear();
  for (int i = 0; i < size; i++) {
    pixels->setPixelColor(i, pixels->Color(red, green, blue));
  }
  pixels->show();
}

//int elapsedTime = 0;

void loop() {
  bool b0, b1, b2, b3, b4;

  // HIGH is 0, LOW is 1 on the inputs
  b0 = !digitalRead(DIGITAL_D0);
  b1 = !digitalRead(DIGITAL_D1);
  b2 = !digitalRead(DIGITAL_D2);
  b3 = !digitalRead(DIGITAL_D3);
  b4 = !digitalRead(DIGITAL_D4);

  digitalWrite(OUTPUT_D0, b0);
  digitalWrite(OUTPUT_D1, b1);
  digitalWrite(OUTPUT_D2, b2);
  digitalWrite(OUTPUT_D3, b3);
  digitalWrite(OUTPUT_D4, b4);


  // bits 3 and 4 reversed
  mode = ((int)b0 << 0) + ((int)b1 << 1) + ((int)b2 << 2) + ((int)b3 << 4) + ((int)b4 << 3);
  Serial.print("Mode: ");
  Serial.println(mode);

  if (mode >= 10 && mode <= 20) {
    farOffGradient(&pixelsFront, NUMPIXELS_FRONT, mode - 10);
    farOffGradient(&pixelsSides, NUMPIXELS_SIDES, mode - 10);
  } else {
    switch (mode) {
      case 0:  // idle
        idleMode(&pixelsFront, NUMPIXELS_FRONT);
        idleMode(&pixelsSides, NUMPIXELS_SIDES);
        break;

      case 1:  // fast flash
        flashFast(false, true, false, &pixelsFront, NUMPIXELS_FRONT);
        flashFast(false, true, false, &pixelsSides, NUMPIXELS_SIDES);
        break;

      case 2:  // drive slowly trigger
        rapidFlash(cyan, &pixelsFront, NUMPIXELS_FRONT, timer);
        rapidFlash(cyan, &pixelsSides, NUMPIXELS_SIDES, timer);
        break;

      case 3:  // full auto mode
        fullAutoGradient(&pixelsFront, NUMPIXELS_FRONT, timer);
        fullAutoGradient(&pixelsSides, NUMPIXELS_SIDES, timer);

        break;

      case 5: // close to reef target
        setFullColor(fullGreen, &pixelsFront, NUMPIXELS_FRONT);
        setFullColor(fullGreen, &pixelsSides, NUMPIXELS_SIDES);
        break;

      case 6: // not close to reef target
        setFullColor(fullRed, &pixelsFront, NUMPIXELS_FRONT);
        setFullColor(fullRed, &pixelsSides, NUMPIXELS_SIDES);
        break;

      default:
        break;
    }
  }


  myTime = millis();
  timer++;
  delay(1);
  //elapsedTime += myTime - millis();
}

void idleMode(Adafruit_NeoPixel *pixels, int size) {
  pixels->clear();

  if (timer > IDLE_BLOCK * IDLE_CYCLE || timer < 0) {
    timer = 0;
  }

  int offset = timer / IDLE_CYCLE;

  for (int i = 0; i < size; i++) {
    uint32_t color;
    int pos = (offset + i) % IDLE_BLOCK;
    if (pos == 0 || pos == 3) {
      color = lowBlue;
    } else if (pos == 1 || pos == 2) {
      color = highBlue;
    } else if (pos == 4 || pos == 7) {
      color = lowGold;
    } else if (pos == 5 || pos == 6) {
      color = highGold;
    } else {
      color = pixels->Color(0, 0, 0);
    }
    pixels->setPixelColor(i, color);
  }

  pixels->show();
}

void flashFast(bool red, bool green, bool blue, Adafruit_NeoPixel *pixels, int size) {
  if (timer < 15) {
    setColor(red, green, blue, pixels, size);
  }

  if (timer < 30 && timer > 15) {
    setColor(false, false, false, pixels, size);
  }

  if (timer > 31 || timer < 0) {
    timer = 0;
  }
}

void setFullColor(uint32_t setColor, Adafruit_NeoPixel *pixels, int size) {
  pixels->clear();
  pixels->fill(setColor, 0);
  pixels->show();
}

void rapidFlash(uint32_t setColor, Adafruit_NeoPixel *pixels, int size, int time) {
  for (int i = 0; i < size; i++) {
    int willSet = (time + i) % 2;


    if (willSet == 0) {
      pixels->setPixelColor(i, setColor);
    }
  }

  pixels->show();
}

// distance goes from 0 to 10, 0 being spot on, 10 being off
void farOffGradient(Adafruit_NeoPixel *pixels, int size, int distance) {
  int red = (int) (255 * (distance / 10.0) + 255 * (1.0 - distance / 10.0));
  int green = 255 * (1.0 - distance / 10.0);
  int blue = (int) (255 * (distance / 10.0) + 255 * (1.0 - distance / 10.0));

  uint32_t setColor = pixels->Color(red, green, blue);


  pixels->fill(setColor, 0);
  pixels->show();
}

void fullAutoGradient(Adafruit_NeoPixel *pixels, int size, int time) {
    double progress = 0.0;

    int redFirst = 0;
    int greenFirst = 0;
    int blueFirst = 0;

    int redSecond = 0;
    int greenSecond = 0;
    int blueSecond = 0;

  for (int i = 0; i < size; i++) {
    time %= 510;

    if (time <= 255) {  // lerp from pink to blue
      progress = time / 255.0;

      redFirst = 255;
      greenFirst = 192;
      blueFirst = 225;

      redSecond = 112;
      greenSecond = 59;
      blueSecond = 231;


    } else {  // lerp from blue to pink
      progress = (time - 255) / 255.0;

      redSecond = 255;
      greenSecond = 192;
      blueSecond = 225;

      redFirst = 112;
      greenFirst = 59;
      blueFirst = 231;
    }


    int redAdjusted = (int)(redFirst * time + redSecond * (1.0 - time));
    int greenAdjusted = (int)(greenFirst * time + greenSecond * (1.0 - time));
    int blueAdjusted = (int)(blueFirst * time + blueSecond * (1.0 - time));

    uint32_t setColor = pixels->Color(redAdjusted, greenAdjusted, blueAdjusted);

    pixels->setPixelColor(i, setColor);

    time++;
  }

  pixels->show();
}
