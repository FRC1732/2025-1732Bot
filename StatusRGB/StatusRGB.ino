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

#define NUMPIXELS_FRONT 24  // number of neopixels in strip
#define NUMPIXELS_SIDES 24  // number of neopixels in strip

#define EYES_START 0 // start of where we turn the eyes red
#define EYES_START_SECOND 3 // second start of where we turn the eyes red
#define EYES_LENGTH 2 // number of pixels we want to turn red for the ram
#define EYES_LENGTH_SECOND 2 // second number of pixels we want to turn red for the ram

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
uint32_t fullBlue = pixelsFront.Color(0, 0, 255);

bool doBlueEyes = false;

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
  pixelsSides.begin();
}

void redEyes(Adafruit_NeoPixel *pixels) { // method that turns the ram eyes red
  uint32_t pickColor = fullRed;
  if (doBlueEyes) {
    uint32_t pickColor = fullBlue;
  }
  pixels->fill(pickColor, EYES_START, EYES_LENGTH);
  pixels->fill(pickColor, EYES_START_SECOND, EYES_LENGTH_SECOND);
}

void setColor(bool red, bool green, bool blue, Adafruit_NeoPixel *pixels, int size) {
  pixels->clear();
  for (int i = 0; i < size; i++) {
    pixels->setPixelColor(i, pixels->Color(INTENSITY * (int)red, INTENSITY * (int)green * .50, INTENSITY * (int)blue));
  }
  if (size == NUMPIXELS_FRONT) {
    redEyes(pixels);
  }
  pixels->show();
}

void setColorInt(int red, int green, int blue, Adafruit_NeoPixel *pixels, int size) {
  pixels->clear();
  for (int i = 0; i < size; i++) {
    pixels->setPixelColor(i, pixels->Color(red, green, blue));
  }
  if (size == NUMPIXELS_FRONT) {
    redEyes(pixels);
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

  /*int doBlue = (int)b3 << 3;
  if (doBlue > 0) {
    doBlueEyes = true;
  } else {
    doBlueEyes = false;
  }*/

  // bits 3 and 4 reversed
  mode = ((int)b0 << 0) + ((int)b1 << 1) + ((int)b2 << 2) + ((int)b3 << 4) + ((int)b4 << 3);
  Serial.print("Mode: ");
  Serial.println(mode);

  if (mode >= 16) {
    Serial.println("Fast Flash");
    flashFast(false, true, false, &pixelsFront, NUMPIXELS_FRONT);
    flashFast(false, true, false, &pixelsSides, NUMPIXELS_SIDES);
  } else {
    switch (mode) {
      case 0:  // idle
        idleMode(&pixelsFront, NUMPIXELS_FRONT);
        idleMode(&pixelsSides, NUMPIXELS_SIDES);
        break;

      case 1:  // mode == speaker
        Serial.println("Speaker Mode");
        setColorInt(255, 90, 200, &pixelsFront, NUMPIXELS_FRONT);
        setColorInt(255, 90, 200, &pixelsSides, NUMPIXELS_SIDES);
        break;

      case 2:  // !hasClearance
        Serial.println("Clearance");
        setColorInt(255, 0, 0, &pixelsFront, NUMPIXELS_FRONT);
        setColorInt(255, 0, 0, &pixelsSides, NUMPIXELS_SIDES);
        break;

      case 3:  // target ready
        setColorInt(0, 125, 0, &pixelsFront, NUMPIXELS_FRONT);
        setColorInt(0, 125, 0, &pixelsSides, NUMPIXELS_SIDES);
        break;

      case 4:  // climbing
        // climberColors(true, true, true, pixelsFront, NUMPIXELS_FRONT);
        // climberColors(true, true, true, pixelsSides, NUMPIXELS_SIDES);
        climberColorsRainbow(&pixelsFront, NUMPIXELS_FRONT);
        climberColorsRainbow(&pixelsSides, NUMPIXELS_SIDES);
        break;
      case 5:  // note seen by intake limelight
        setColorInt(255, 50, 0, &pixelsFront, NUMPIXELS_FRONT);
        setColorInt(255, 50, 0, &pixelsSides, NUMPIXELS_SIDES);
        break;

      default:
        break;
    }


    // pixelsFront.show();

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

  if (size == NUMPIXELS_FRONT) {
    redEyes(pixels);
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

int currentlyFlashingFront[NUMPIXELS_FRONT];
int currentlyFlashingSide[NUMPIXELS_SIDES];

void climberColors(bool red, bool green, bool blue, Adafruit_NeoPixel *pixels, int size) {
  if (myTime % 15 == 0) {
    int pickTable[size] = {};
    // there isnt a better way to do this sadly
    for (int i = 0; i < size; i++) {
      if (size == NUMPIXELS_FRONT) {
        pickTable[i] = currentlyFlashingFront[i];
      } else {
        pickTable[i] = currentlyFlashingSide[i];
      }
    }

    pixels->clear();
    for (int i = 0; i < size; i++) {
      int rng = random(1, 25);
      if (rng == 1 && pickTable[i] == 0) {
        pickTable[i] = 250;
      } else if (pickTable[i] > 0) {
        pickTable[i] = max(0, pickTable[i] - 20);
      }
      uint32_t setColor = pixels->Color(red * pickTable[i], green * pickTable[i], blue * pickTable[i]);
      pixels->setPixelColor(i, setColor);
    }

    if (size == NUMPIXELS_FRONT) {
      redEyes(pixels);
    }

    pixels->show();

    // and convert the table back
    for (int i = 0; i < size; i++) {
      if (size == NUMPIXELS_FRONT) {
        currentlyFlashingFront[i] = pickTable[i];
      } else {
        currentlyFlashingSide[i] = pickTable[i];
      }
    }
  }
}

// RAINBOW verison of the climber
int currentlyFlashingColorFront[NUMPIXELS_FRONT][3];
int currentlyFlashingColorSide[NUMPIXELS_SIDES][3];

void climberColorsRainbow(Adafruit_NeoPixel *pixels, int size) {
  if (myTime % 10 == 0) {
    int pickTable[size] = {};
    int pickTableColor[size][3] = {};

    for (int i = 0; i < size; i++) {
      if (size == NUMPIXELS_FRONT) {
        pickTable[i] = currentlyFlashingFront[i];
        pickTableColor[i][0] = currentlyFlashingColorFront[i][0];
        pickTableColor[i][1] = currentlyFlashingColorFront[i][1];
        pickTableColor[i][2] = currentlyFlashingColorFront[i][2];
      } else {
        pickTable[i] = currentlyFlashingSide[i];
        pickTableColor[i][0] = currentlyFlashingColorSide[i][0];
        pickTableColor[i][1] = currentlyFlashingColorSide[i][1];
        pickTableColor[i][2] = currentlyFlashingColorSide[i][2];
      }
    }

    pixels->clear();
    for (int i = 0; i < size; i++) {
      int rng = random(1, 15);

      if (rng == 1 && pickTable[i] == 0) {
        pickTable[i] = 250;
        pickTableColor[i][0] = random(5, 255);
        pickTableColor[i][1] = random(5, 255);
        pickTableColor[i][2] = random(5, 255);

      } else if (pickTable[i] > 0) {
        pickTable[i] = max(0, pickTable[i] - 20);
      }
      
      uint32_t setColor = pixels->Color(pickTableColor[i][0] * (pickTable[i] / 250.0), pickTableColor[i][1] * (pickTable[i] / 250.0), pickTableColor[i][2] * (pickTable[i] / 250.0));
      pixels->setPixelColor(i, setColor);
    }
    if (size == NUMPIXELS_FRONT) {
      redEyes(pixels);
    }
    pixels->show();

    for (int i = 0; i < size; i++) {
      if (size == NUMPIXELS_FRONT) {
        currentlyFlashingFront[i] = pickTable[i];
        currentlyFlashingColorFront[i][0] = pickTableColor[i][0];
        currentlyFlashingColorFront[i][1] = pickTableColor[i][1];
        currentlyFlashingColorFront[i][2] = pickTableColor[i][2];
      } else {
        currentlyFlashingSide[i] = pickTable[i];
        currentlyFlashingColorSide[i][0] = pickTableColor[i][0];
        currentlyFlashingColorSide[i][1] = pickTableColor[i][1];
        currentlyFlashingColorSide[i][2] = pickTableColor[i][2];
      }
    }
  }
}


// int currentlyFlashingColorOld[NUMPIXELS][3];
// int waitTime = 50;
// bool colorDirection = true;

// // different take on the idle state, currently broken
// void topperLine() {
//   if (myTime % waitTime == 0) {
//     if (colorDirection == false) {
//       waitTime++;
//     } else {
//       waitTime--;
//     }

//     if (waitTime == 1 || waitTime == 55) {
//       colorDirection = !colorDirection;
//     }

//     bool doColor = false;
//     if (timer >= 125 && waitTime <= 60) {
//       timer = 0;
//       doColor = true;
//     }
//     pixelsFront.clear();

//     for (int i = 0; i < NUMPIXELS; i++) {
//       if (currentlyFlashingColor[i][0] == 0 && currentlyFlashingColor[i][2] == 0) {
//           currentlyFlashingColor[i][0] = 0;
//           currentlyFlashingColor[i][1] = 0;
//           currentlyFlashingColor[i][2] = 255;
//       }
//     }

//     for (int i = 0; i < NUMPIXELS; i++) {
//       currentlyFlashingColorOld[i][0] = currentlyFlashingColor[i][0];
//       currentlyFlashingColorOld[i][1] = currentlyFlashingColor[i][1];
//       currentlyFlashingColorOld[i][2] = currentlyFlashingColor[i][2];
//     }

//     for (int i = 0; i < NUMPIXELS ; i++) {
//       int putPosition = i;

//       if (putPosition == 0) {
//         if (!doColor) {
//           currentlyFlashingColor[putPosition][0] = 0;
//           currentlyFlashingColor[putPosition][1] = 0;
//           currentlyFlashingColor[putPosition][2] = 255;
//         } else {
//           currentlyFlashingColor[putPosition][0] = 255;
//           currentlyFlashingColor[putPosition][1] = 125;
//           currentlyFlashingColor[putPosition][2] = 0;
//         }

//       } else {
//         shiftColor(putPosition, 1);
//       }

//       pixelsFront.setPixelColor(putPosition, pixelsFront.Color(currentlyFlashingColor[putPosition][0], currentlyFlashingColor[putPosition][1], currentlyFlashingColor[putPosition][2]));
//     }
//     pixelsFront.show();
//   }
// }

// void shiftColor(int pos, int direction) {
//   currentlyFlashingColor[pos][0] = currentlyFlashingColorOld[pos - direction][0];
//   currentlyFlashingColor[pos][1] = currentlyFlashingColorOld[pos - direction][1];
//   currentlyFlashingColor[pos][2] = currentlyFlashingColorOld[pos - direction][2];
// }
