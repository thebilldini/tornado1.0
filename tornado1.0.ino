#include <Bounce2.h>
#include <FastLED.h>
#include <Adafruit_NeoPixel.h>

#define RED_BUTTON_INPUT 2
#define RED_BUTTON_OUTPUT 10
#define RED_BUTTON_LED 11
#define BLUE_BUTTON_INPUT 3
#define BLUE_BUTTON_OUTPUT 9
#define BLUE_BUTTON_LED 12
#define DATA_PIN_1 5  //red rope
#define DATA_PIN_2 6  //blue rope
#define DATA_PIN_3 7  //small cloud
#define DATA_PIN_4 8  //large cloud
#define CLOCK_PIN 13

#define NUM_CHASE_LEDS 43
#define NUM_CHASE 6
const int CHASE_GAP = 7;
#define NUM_LEDS_SMALL_CLOUD 49
#define NUM_LEDS_LARGE_CLOUD 56
#define BRIGHTNESS 255
#define BLUE_H 150
#define BLUE_S 255
#define RED_H 0
#define RED_S 255
#define ALTERNATOR_PERIOD 250
#define CHASE_PERIOD 25
#define MIN_BRIGHTNESS 0
#define MAX_BRIGHTNESS 255
#define HALF_BRIGHTNESS 127
#define CLOUD_STANDBY_TIME 3000
#define CLOUD_FIRST_FLASH_TIME 300
#define CLOUD_FIRST_BREAK_TIME 150
#define CLOUD_SECOND_FLASH_TIME 400
#define CLOUD_SECOND_BREAK_TIME 150
#define CLOUD_THIRD_FLASH_TIME 200
#define MAX_TME_BETWEEN_STRIKES 7000
#define SLOW_OFF 6
#define FAST_OFF 2
#define FAST_DECAY_SPEED 51
#define SLOW_DECAY_SPEED 20

CRGB ledsRed[NUM_CHASE_LEDS];
CRGB ledsBlue[NUM_CHASE_LEDS];
Adafruit_NeoPixel smallCloud(NUM_LEDS_SMALL_CLOUD, DATA_PIN_3, NEO_GRBW + NEO_KHZ400);
Adafruit_NeoPixel largeCloud(NUM_LEDS_LARGE_CLOUD, DATA_PIN_4, NEO_GRBW + NEO_KHZ400);

enum STATES {
  OFF,
  RED_ACTIVE,    // Red rope + small cloud
  BLUE_ACTIVE,   // Blue rope + large cloud
  BOTH_ACTIVE    // Both ropes + both clouds
};

enum BUTTONS { RED_BUTTON,
               BLUE_BUTTON,
               NUM_BUTTONS };

enum CLOUDS { SMALL_CLOUD,
              LARGE_CLOUD,
              NUM_CLOUDS };

enum CLOUD_STATES { STANDBY,
                    FIRST_FLASH,
                    FIRST_BREAK,
                    SECOND_FLASH,
                    SECOND_BREAK,
                    THIRD_FLASH,
                    NUM_CLOUD_STATES,
                    CLOUD_OFF };

const uint32_t CLOUD_STATE_TIMES[NUM_CLOUD_STATES] = { CLOUD_STANDBY_TIME, CLOUD_FIRST_FLASH_TIME, CLOUD_FIRST_BREAK_TIME, CLOUD_SECOND_FLASH_TIME, CLOUD_SECOND_BREAK_TIME, CLOUD_THIRD_FLASH_TIME };
int cloudBrightness[NUM_CLOUDS] = { MIN_BRIGHTNESS, MIN_BRIGHTNESS };
uint32_t cloudTimer[NUM_CLOUDS] = { 0, 0 };
uint32_t TIMEOUT;
int cloudState[NUM_CLOUDS] = { STANDBY, STANDBY };
int chaseState;
int alternator = 1;
volatile uint8_t LEDIndex;
int state = OFF;
const uint8_t BUTTON_PINS[NUM_BUTTONS] = { RED_BUTTON_INPUT, BLUE_BUTTON_INPUT };
Bounce* buttons = new Bounce[NUM_BUTTONS];
uint32_t initialMillis, currentMillis, timeDifference, lastIncrement;
const float ALTERNATOR_FREQUENCY = ALTERNATOR_PERIOD / 2;
const float CHASE_FREQUENCY = CHASE_PERIOD / 2;
bool chaseUpdate = true;
bool redChase, blueChase;

// Place these globals near the top of your file:
unsigned long cloudFlickerStart[NUM_CLOUDS] = {0, 0};
unsigned long cloudFlickerDuration[NUM_CLOUDS] = {0, 0};
bool cloudFlickerActive[NUM_CLOUDS] = {false, false};
uint8_t cloudFlickerR[NUM_CLOUDS] = {0, 0};
uint8_t cloudFlickerG[NUM_CLOUDS] = {0, 0};
uint8_t cloudFlickerB[NUM_CLOUDS] = {0, 0};
uint8_t cloudFlickerFade[NUM_CLOUDS] = {0, 0};
unsigned long cloudFlickerFadeDuration[NUM_CLOUDS] = {0, 0}; // NEW: random fade duration

// Add at the top (after other globals)
unsigned long fillStartMillis = 0;
int fillIndex = 0;
bool fillBlanked = false;
const unsigned long FILL_STEP_TIME = 20; // ms per LED fill
const unsigned long FILL_HOLD_TIME = 200; // ms to hold before blanking

void setup() {
  Serial.begin(115200);
  //setup for LEDs
  // FastLED.setBrightness(BRIGHTNESS);

  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(RED_BUTTON_OUTPUT, OUTPUT);
  pinMode(RED_BUTTON_LED, OUTPUT);
  pinMode(BLUE_BUTTON_OUTPUT, OUTPUT);
  pinMode(BLUE_BUTTON_LED, OUTPUT);
  delay(100);
  //millis was running slow for unclear reasons and so timeout  halved to match
  TIMEOUT = (30000 * !digitalRead(A1) + 60000 * !digitalRead(A2) + 90000 * !digitalRead(A3) + 120000 * !digitalRead(A4)) / 2;
  FastLED.addLeds<TM1804, DATA_PIN_1, RGB>(ledsRed, NUM_CHASE_LEDS);
  FastLED.addLeds<TM1804, DATA_PIN_2, RGB>(ledsBlue, NUM_CHASE_LEDS);
  smallCloud.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  smallCloud.show();   // Turn OFF all pixels ASAP
  smallCloud.setBrightness(MAX_BRIGHTNESS);
  largeCloud.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  largeCloud.show();   // Turn OFF all pixels ASAP
  largeCloud.setBrightness(MAX_BRIGHTNESS);
  //setup buttons
  for (int i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].attach(BUTTON_PINS[i], INPUT_PULLUP);  //setup the bounce instance for the current button
    buttons[i].interval(25);                          // interval in ms
  }

  //analog input pin 0 isn't connected, so the random analog
  //noise causes randomSeed() to generate different
  // seed numbers each time the sketch runs.
  randomSeed(analogRead(0));
  initialMillis = millis();
  Serial.println(initialMillis);
  Serial.println(TIMEOUT);
}

void loop() {
  updateButtons();
  updateState();
  runInteractive();
}

void updateButtons() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    // Update the Bounce instance :
    buttons[i].update();
  }
}

void updateState() {
  bool tempRed = buttons[RED_BUTTON].fell();
  bool tempBlue = buttons[BLUE_BUTTON].fell();
  currentMillis = millis();
  timeDifference = currentMillis - initialMillis;

  // TIMEOUT: turn off everything
  if (timeDifference > TIMEOUT && state != OFF) {
    state = OFF;
    initialMillis = currentMillis;
    redChase = false;
    blueChase = false;
  }

  // State transitions
  switch (state) {
    case OFF:
      if (tempRed && tempBlue) {
        state = BOTH_ACTIVE;
        initialMillis = currentMillis;
      } else if (tempRed) {
        state = RED_ACTIVE;
        initialMillis = currentMillis;
      } else if (tempBlue) {
        state = BLUE_ACTIVE;
        initialMillis = currentMillis;
      }
      break;
    case RED_ACTIVE:
      if (tempBlue) {
        state = BOTH_ACTIVE;
        initialMillis = currentMillis;
      }
      break;
    case BLUE_ACTIVE:
      if (tempRed) {
        state = BOTH_ACTIVE;
        initialMillis = currentMillis;
      }
      break;
    case BOTH_ACTIVE:
      // Optionally, you can allow going back to single states if a button is released
      // Or require timeout to return to OFF
      break;
  }

  // Set chase and cloud states based on main state
  switch (state) {
    case OFF:
      redChase = false;
      blueChase = false;
      cloudState[SMALL_CLOUD] = STANDBY;
      cloudState[LARGE_CLOUD] = STANDBY;
      break;
    case RED_ACTIVE:
      redChase = true;
      blueChase = false;
      cloudState[SMALL_CLOUD] = FIRST_FLASH; // or your desired active state
      cloudState[LARGE_CLOUD] = STANDBY;
      break;
    case BLUE_ACTIVE:
      redChase = false;
      blueChase = true;
      cloudState[SMALL_CLOUD] = STANDBY;
      cloudState[LARGE_CLOUD] = FIRST_FLASH; // or your desired active state
      break;
    case BOTH_ACTIVE:
      redChase = true;
      blueChase = true;
      cloudState[SMALL_CLOUD] = FIRST_FLASH; // or a special state for both
      cloudState[LARGE_CLOUD] = FIRST_FLASH;
      break;
  }
}

void runInteractive() {
  alternate();
  if (redChase || blueChase) {
    gradualFillEffect();
  } else {
    chase();
  }
  cloudLights();
  blowers();
  FastLED.show();
}
//alternate updates the pattern for a strand in the off phase
void alternate() {
  //red alternating
  if (!redChase) {
    for (int LED = 0; LED < NUM_CHASE_LEDS; LED++) {
      if (LED % 2 == 0 && chaseState == 0 || LED % 2 == 1 && chaseState == 1) {
        ledsRed[LED] = CHSV(RED_H, RED_S, BRIGHTNESS);
      } else {
        ledsRed[LED] = CRGB::Black;
      }
    }
  }
  //blue alternating
  if (!blueChase) {
    for (int LED = 0; LED < NUM_CHASE_LEDS; LED++) {
      if (LED % 2 == 0 && chaseState == 0 || LED % 2 == 1 && chaseState == 1) {
        ledsBlue[LED] = CHSV(BLUE_H, BLUE_S, BRIGHTNESS);
      } else {
        ledsBlue[LED] = CRGB::Black;
      }
    }
  }
  //update alternator to change even/odd leds
  if ((timeDifference) % ALTERNATOR_PERIOD <= ALTERNATOR_FREQUENCY && alternator) {
    chaseState = !chaseState;
    alternator = !alternator;
  } else if ((timeDifference) % ALTERNATOR_PERIOD > ALTERNATOR_FREQUENCY && !alternator) {
    alternator = !alternator;
  }
}
//chase updates where the chase pattern when the strand in question is in an active phase
void chase() {
  if ((timeDifference) % CHASE_PERIOD < CHASE_FREQUENCY && chaseUpdate) {
    //reds go in the opposite direction of blue
    if (redChase) {
      ledsRed[map((LEDIndex) /**/ % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CRGB::Black;
      ledsRed[map((LEDIndex + 1) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, HALF_BRIGHTNESS);
      ledsRed[map((LEDIndex + 2) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, BRIGHTNESS);
      ledsRed[map((LEDIndex + 3) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, HALF_BRIGHTNESS);

      ledsRed[map((LEDIndex + 1 * CHASE_GAP /**/) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CRGB::Black;
      ledsRed[map((LEDIndex + 1 * CHASE_GAP + 1) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, HALF_BRIGHTNESS);
      ledsRed[map((LEDIndex + 1 * CHASE_GAP + 2) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, BRIGHTNESS);
      ledsRed[map((LEDIndex + 1 * CHASE_GAP + 3) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, HALF_BRIGHTNESS);

      ledsRed[map((LEDIndex + 2 * CHASE_GAP /**/) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CRGB::Black;
      ledsRed[map((LEDIndex + 2 * CHASE_GAP + 1) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, HALF_BRIGHTNESS);
      ledsRed[map((LEDIndex + 2 * CHASE_GAP + 2) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, BRIGHTNESS);
      ledsRed[map((LEDIndex + 2 * CHASE_GAP + 3) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, HALF_BRIGHTNESS);

      ledsRed[map((LEDIndex + 3 * CHASE_GAP /**/) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CRGB::Black;
      ledsRed[map((LEDIndex + 3 * CHASE_GAP + 1) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, HALF_BRIGHTNESS);
      ledsRed[map((LEDIndex + 3 * CHASE_GAP + 2) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, BRIGHTNESS);
      ledsRed[map((LEDIndex + 3 * CHASE_GAP + 3) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, HALF_BRIGHTNESS);

      ledsRed[map((LEDIndex + 4 * CHASE_GAP /**/) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CRGB::Black;
      ledsRed[map((LEDIndex + 4 * CHASE_GAP + 1) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, HALF_BRIGHTNESS);
      ledsRed[map((LEDIndex + 4 * CHASE_GAP + 2) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, BRIGHTNESS);
      ledsRed[map((LEDIndex + 4 * CHASE_GAP + 3) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, HALF_BRIGHTNESS);

      ledsRed[map((LEDIndex + 5 * CHASE_GAP /**/) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CRGB::Black;
      ledsRed[map((LEDIndex + 5 * CHASE_GAP + 1) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, HALF_BRIGHTNESS);
      ledsRed[map((LEDIndex + 5 * CHASE_GAP + 2) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, BRIGHTNESS);
      ledsRed[map((LEDIndex + 5 * CHASE_GAP + 3) % NUM_CHASE_LEDS, 0, NUM_CHASE_LEDS - 1, NUM_CHASE_LEDS - 1, 0)] = CHSV(RED_H, RED_S, HALF_BRIGHTNESS);
    }
    if (blueChase) {
      ledsBlue[(LEDIndex) /**/ % NUM_CHASE_LEDS] = CRGB::Black;
      ledsBlue[(LEDIndex + 1) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, HALF_BRIGHTNESS);
      ledsBlue[(LEDIndex + 2) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, BRIGHTNESS);
      ledsBlue[(LEDIndex + 3) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, HALF_BRIGHTNESS);

      ledsBlue[(LEDIndex + 1 * CHASE_GAP /**/) % NUM_CHASE_LEDS] = CRGB::Black;
      ledsBlue[(LEDIndex + 1 * CHASE_GAP + 1) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, HALF_BRIGHTNESS);
      ledsBlue[(LEDIndex + 1 * CHASE_GAP + 2) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, BRIGHTNESS);
      ledsBlue[(LEDIndex + 1 * CHASE_GAP + 3) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, HALF_BRIGHTNESS);

      ledsBlue[(LEDIndex + 2 * CHASE_GAP /**/) % NUM_CHASE_LEDS] = CRGB::Black;
      ledsBlue[(LEDIndex + 2 * CHASE_GAP + 1) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, HALF_BRIGHTNESS);
      ledsBlue[(LEDIndex + 2 * CHASE_GAP + 2) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, BRIGHTNESS);
      ledsBlue[(LEDIndex + 2 * CHASE_GAP + 3) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, HALF_BRIGHTNESS);

      ledsBlue[(LEDIndex + 3 * CHASE_GAP) % NUM_CHASE_LEDS] = CRGB::Black;
      ledsBlue[(LEDIndex + 3 * CHASE_GAP + 1) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, HALF_BRIGHTNESS);
      ledsBlue[(LEDIndex + 3 * CHASE_GAP + 2) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, BRIGHTNESS);
      ledsBlue[(LEDIndex + 3 * CHASE_GAP + 3) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, HALF_BRIGHTNESS);

      ledsBlue[(LEDIndex + 4 * CHASE_GAP /**/) % NUM_CHASE_LEDS] = CRGB::Black;
      ledsBlue[(LEDIndex + 4 * CHASE_GAP + 1) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, HALF_BRIGHTNESS);
      ledsBlue[(LEDIndex + 4 * CHASE_GAP + 2) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, BRIGHTNESS);
      ledsBlue[(LEDIndex + 4 * CHASE_GAP + 3) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, HALF_BRIGHTNESS);

      ledsBlue[(LEDIndex + 5 * CHASE_GAP /**/) % NUM_CHASE_LEDS] = CRGB::Black;
      ledsBlue[(LEDIndex + 5 * CHASE_GAP + 1) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, HALF_BRIGHTNESS);
      ledsBlue[(LEDIndex + 5 * CHASE_GAP + 2) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, BRIGHTNESS);
      ledsBlue[(LEDIndex + 5 * CHASE_GAP + 3) % NUM_CHASE_LEDS] = CHSV(BLUE_H, BLUE_S, HALF_BRIGHTNESS);
    }
    chaseUpdate = false;
  }
  //update index
  else if (timeDifference % CHASE_PERIOD >= CHASE_FREQUENCY && !chaseUpdate) {
    chaseUpdate = true;
    LEDIndex = (LEDIndex + 1) % NUM_CHASE_LEDS;
  }
}
//takes a cloud number and updates that clouds lighting
void colorSet(int cloudNum) {
  if (cloudNum == SMALL_CLOUD) {
    for (int i = 0; i < smallCloud.numPixels(); i++) {                                       // For each pixel in strip...
      smallCloud.setPixelColor(i, smallCloud.Color(0, 0, 0, cloudBrightness[SMALL_CLOUD]));  //  Set pixel's color (in RAM)
    }
    // Serial.print(cloudBrightness[SMALL_CLOUD]);
    // Serial.print(" ");
    smallCloud.show();
  }

  if (cloudNum == LARGE_CLOUD) {
    for (int i = 0; i < largeCloud.numPixels(); i++) {                                       // For each pixel in strip...
      largeCloud.setPixelColor(i, largeCloud.Color(0, 0, 0, cloudBrightness[LARGE_CLOUD]));  //  Set pixel's color (in RAM)
    }
    largeCloud.show();
    // Serial.println(cloudBrightness[LARGE_CLOUD]);
  }
}
//adjusts the brightness of the clouds and calls the update function
void cloudLights() {
  for (int i = 0; i < NUM_CLOUDS; i++) {
    switch (cloudState[i]) {
      case STANDBY:
        if (cloudBrightness[i] > MIN_BRIGHTNESS && currentMillis - lastIncrement > SLOW_OFF) {
          cloudBrightness[i] = cloudBrightness[i] - FAST_DECAY_SPEED;
          cloudBrightness[i] = max(cloudBrightness[i], 0);
          lastIncrement = currentMillis;
        }
        colorSet(i);
        cloudFlickerActive[i] = false;
        break;

      case FIRST_FLASH:
      case SECOND_FLASH:
      case THIRD_FLASH:
      case FIRST_BREAK:
      case SECOND_BREAK: {
        cloudBrightness[i] = MAX_BRIGHTNESS;

        // Randomly start a flicker (low probability each loop)
        if (!cloudFlickerActive[i] && random(0, 100) < 2) { // ~2% chance per loop
          cloudFlickerActive[i] = true;
          cloudFlickerStart[i] = millis();
          cloudFlickerDuration[i] = random(100, 300); // Flicker color duration (short)
          cloudFlickerFadeDuration[i] = random(150, 800); // Random fade-to-white duration

          // Set flicker color: red for small, blue for large
          if (i == SMALL_CLOUD) {
            cloudFlickerR[i] = MAX_BRIGHTNESS;
            cloudFlickerG[i] = 0;
            cloudFlickerB[i] = 0;
          } else {
            cloudFlickerR[i] = 0;
            cloudFlickerG[i] = 0;
            cloudFlickerB[i] = MAX_BRIGHTNESS;
          }
          cloudFlickerFade[i] = 255;
        }

        // If flicker is active, handle color and fade
        unsigned long elapsed = millis() - cloudFlickerStart[i];
        if (cloudFlickerActive[i]) {
          if (elapsed > (cloudFlickerDuration[i] + cloudFlickerFadeDuration[i])) {
            cloudFlickerActive[i] = false;
            cloudFlickerFade[i] = 0;
          } else if (elapsed > cloudFlickerDuration[i]) {
            // Fade out over random duration
            unsigned long fadeElapsed = elapsed - cloudFlickerDuration[i];
            cloudFlickerFade[i] = map(fadeElapsed, 0, cloudFlickerFadeDuration[i], 255, 0);
          } else {
            cloudFlickerFade[i] = 255;
          }
        }

        // Set color: mostly white, but blend in flicker if active
        uint8_t r = MAX_BRIGHTNESS, g = MAX_BRIGHTNESS, b = MAX_BRIGHTNESS;
        if (cloudFlickerActive[i] && cloudFlickerFade[i] > 0) {
          r = ((uint16_t)cloudFlickerR[i] * cloudFlickerFade[i] + MAX_BRIGHTNESS * (255 - cloudFlickerFade[i])) / 255;
          g = ((uint16_t)cloudFlickerG[i] * cloudFlickerFade[i] + MAX_BRIGHTNESS * (255 - cloudFlickerFade[i])) / 255;
          b = ((uint16_t)cloudFlickerB[i] * cloudFlickerFade[i] + MAX_BRIGHTNESS * (255 - cloudFlickerFade[i])) / 255;
        }

        if (i == SMALL_CLOUD) {
          for (int p = 0; p < smallCloud.numPixels(); p++) {
            smallCloud.setPixelColor(p, smallCloud.Color(r, g, b, 0));
          }
          smallCloud.show();
        } else {
          for (int p = 0; p < largeCloud.numPixels(); p++) {
            largeCloud.setPixelColor(p, largeCloud.Color(r, g, b, 0));
          }
          largeCloud.show();
        }
        break;
      }

      case CLOUD_OFF:
        cloudBrightness[i] = MIN_BRIGHTNESS;
        colorSet(i);
        cloudFlickerActive[i] = false;
        break;
    }
    Serial.print(i);
    Serial.print("<-cloud brightness->");
    Serial.println(cloudBrightness[i]);
  }
}
//changes the blowers and button LEDS in accordance with the current state of their corresponding chase.
void blowers() {
  digitalWrite(RED_BUTTON_OUTPUT, redChase);
  digitalWrite(RED_BUTTON_LED, !redChase);
  digitalWrite(BLUE_BUTTON_OUTPUT, blueChase);
  digitalWrite(BLUE_BUTTON_LED, !blueChase);
}

// New function for gradual fill effect
void gradualFillEffect() {
  static unsigned long lastStep = 0;
  static int maxFill = NUM_CHASE_LEDS;
  unsigned long now = millis();

  // Reset if state just became active
  if (!fillBlanked && fillIndex == 0) {
    fillStartMillis = now;
  }

  // Gradually fill
  if (!fillBlanked && fillIndex < maxFill && now - lastStep > FILL_STEP_TIME) {
    lastStep = now;
    fillIndex++;
    // Fill red
    if (redChase) {
      for (int i = 0; i < fillIndex; i++) {
        ledsRed[i] = CHSV(RED_H, RED_S, BRIGHTNESS);
      }
      for (int i = fillIndex; i < maxFill; i++) {
        ledsRed[i] = CRGB::Black;
      }
    }
    // Fill blue
    if (blueChase) {
      for (int i = 0; i < fillIndex; i++) {
        ledsBlue[i] = CHSV(BLUE_H, BLUE_S, BRIGHTNESS);
      }
      for (int i = fillIndex; i < maxFill; i++) {
        ledsBlue[i] = CRGB::Black;
      }
    }
  }

  // Hold full for a moment, then blank all at once
  if (!fillBlanked && fillIndex >= maxFill && now - fillStartMillis > (FILL_STEP_TIME * maxFill + FILL_HOLD_TIME)) {
    fillBlanked = true;
    // Blank all
    for (int i = 0; i < maxFill; i++) {
      if (redChase) ledsRed[i] = CRGB::Black;
      if (blueChase) ledsBlue[i] = CRGB::Black;
    }
    lastStep = now;
  }

  // After blank, reset for next cycle
  if (fillBlanked && now - lastStep > FILL_HOLD_TIME) {
    fillIndex = 0;
    fillBlanked = false;
    fillStartMillis = now;
  }
}