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
#define BLUE_H 140
#define BLUE_S 140
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
  chase();
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
      //standby slowly decays the cloud to 0
      case STANDBY:
        if (cloudBrightness[i] > MIN_BRIGHTNESS && currentMillis - lastIncrement > SLOW_OFF) {
          cloudBrightness[i] = cloudBrightness[i] - FAST_DECAY_SPEED;
          cloudBrightness[i] = max(cloudBrightness[i], 0);
          lastIncrement = currentMillis;
        }
        colorSet(i);
        break;
        //first flash quickly turns the cloud on
      case FIRST_FLASH:
        if (cloudBrightness[i] < MAX_BRIGHTNESS) {
          cloudBrightness[i] = MAX_BRIGHTNESS;
        }
        colorSet(i);
        break;
        //split second quickly decays the cloud
      case FIRST_BREAK:
        if (cloudBrightness[i] > MIN_BRIGHTNESS /*&& currentMillis - lastIncrement > FAST_OFF*/) {
          cloudBrightness[i] = cloudBrightness[i] - SLOW_DECAY_SPEED;
          cloudBrightness[i] = max(cloudBrightness[i], 0);
          // lastIncrement = currentMillis;
        }
        colorSet(i);
        break;
        //second flash quickly turns the cloud on
      case SECOND_FLASH:
        if (cloudBrightness[i] < MAX_BRIGHTNESS) {
          cloudBrightness[i] = MAX_BRIGHTNESS;
        }
        colorSet(i);
        break;

      case SECOND_BREAK:
        if (cloudBrightness[i] > MIN_BRIGHTNESS /*&& currentMillis - lastIncrement > FAST_OFF*/) {
          cloudBrightness[i] = cloudBrightness[i] - SLOW_DECAY_SPEED;
          cloudBrightness[i] = max(cloudBrightness[i], 0);
          // lastIncrement = currentMillis;
        }
        colorSet(i);
        break;
        //third flash quickly turns the cloud on
      case THIRD_FLASH:
        if (cloudBrightness[i] < MAX_BRIGHTNESS) {
          cloudBrightness[i] = MAX_BRIGHTNESS;
        }
        colorSet(i);
        break;
      case CLOUD_OFF:
        cloudBrightness[i] = MIN_BRIGHTNESS;
        colorSet(i);
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