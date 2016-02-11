#include <Energia.h>
#include <SPI.h>
#include <RTCplus.h>

//#define DEBUG

// ------------------------------ Time vars ------------------------------------
RealTimeClock rtc;
typedef struct {
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
} time_t;
static time_t currentTime = {0, 0, 0};
static bool minutesBlink = false;
static bool hoursBlink = false;
static volatile bool secondsTick = false;
// -----------------------------------------------------------------------------

// -------------------------- Display vars -------------------------------------
static const uint8_t DISPLAY_LATCH_PIN = P1_4;
static const uint32_t DIGITS_SETUP_MODE_BLINK_INTERVAL_MS = 300;
#define MINUTES_UNITS_ANODE     0
#define MINUTES_DECIMALS_ANODE  1
#define HOURS_UNITS_ANODE       2
#define HOURS_DECIMALS_ANODE    3
#define DISPLAY_DOT_INDEX       10
#define DISPLAY_BLANK_INDEX     11

static const uint8_t ANODE_DATA[] = {
  0b00000001,
  0b00000010,
  0b00000100,
  0b00001000
};

//PGFEDCBA
static const uint8_t SEGMENT_DATA[] = {
  0b11000000,  // 0
  0b11111001,  // 1
  0b10100100,  // 2
  0b10110000,  // 3
  0b10011001,  // 4
  0b10010010,  // 5
  0b10000010,  // 6
  0b11111000,  // 7
  0b10000000,  // 8
  0b10010000,  // 9
  0b01111111,  //dot
  0b11111111   //blank
};
// -----------------------------------------------------------------------------

// ------------------------------ Keys vars ------------------------------------
static const uint8_t MENU_KEY = PUSH2;
static const uint32_t SHORT_KEYPRESS_MIN_THRESHOLD_MS = 20;
static const uint32_t SHORT_KEYPRESS_MAX_THRESHOLD_MS = 800;
static const uint32_t LONG_KEYPRESS_MAX_THRESHOLD_MS = 1000;
typedef enum {
  PRESSED,
  RELEASED
} KeyState;
typedef enum {
  NO_KEYPRESS_EVENT,
  SHORT_KEYPRESS_EVENT,
  LONG_KEYPRESS_EVENT
} KeyEvent;
typedef struct {
  KeyState state;
  KeyEvent event;
  uint32_t timestamp;
} key_event_t;

static key_event_t keyEvent = {RELEASED, NO_KEYPRESS_EVENT, 0};
static KeyEvent lastKeyEvent = NO_KEYPRESS_EVENT;
// -----------------------------------------------------------------------------

// --------------------------- Sleep vars --------------------------------------
static bool wakeUpFlag = false;
static const uint32_t SLEEP_INTERVAL_MS = 100;
static const uint32_t INACTIVITY_SLEEP_INTERVAL_MS = 4000;
// -----------------------------------------------------------------------------

// --------------------------- FSM vars ----------------------------------------
enum ClockState {
  DISPLAY_STATE,
  MINUTES_SETUP_STATE,
  HOURS_SETUP_STATE,
};
static uint8_t nextState = DISPLAY_STATE;
// -----------------------------------------------------------------------------

// -------------------------- Time handling ------------------------------------
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void) {
  rtc++;

  #ifndef RTCSUBSECONDS
  secondsTick = !secondsTick;
  #else
  #error "Don't know how to handle subsecond ticks yet!!!"
  #endif
}

static time_t* getRtcTime(RealTimeClock* rtc, time_t* currentTime) {
  currentTime->seconds = rtc->RTC_sec;
  currentTime->minutes = rtc->RTC_min;
  currentTime->hours = rtc->RTC_hr;

  return currentTime;
}
// -----------------------------------------------------------------------------

// ---------------------- Display handling -------------------------------------
static void displaySegmentData(uint8_t data, uint8_t digit) {
  digitalWrite(DISPLAY_LATCH_PIN, LOW);
  SPI.transfer(data);
  SPI.transfer(digit);
  digitalWrite(DISPLAY_LATCH_PIN, HIGH);
}

static void blankDisplay() {
  displaySegmentData(SEGMENT_DATA[DISPLAY_BLANK_INDEX], 0);
}

static void displayTime(time_t* currentTime, bool blinkMinutes, bool blinkHours) {
  static uint32_t lastBlinkTime = 0;
  static bool blinkTicker = false;

  if(millis() - lastBlinkTime >= DIGITS_SETUP_MODE_BLINK_INTERVAL_MS) {
    blinkTicker = !blinkTicker;
    lastBlinkTime = millis();
  }

  // extract minutes/hours digit values for indexing the segment data arrays
  uint8_t minutesUnitsDigitIndex = currentTime->minutes % 10;
  uint8_t minutesDecimalsDigitIndex = currentTime->minutes / 10;
  uint8_t hoursUnitsDigitIndex = currentTime->hours % 10;
  uint8_t hoursDecimalsDigitIndex = currentTime->hours / 10;

  // minutes units display
  displaySegmentData(
    SEGMENT_DATA[minutesUnitsDigitIndex],
    (blinkMinutes && blinkTicker) ?
    (ANODE_DATA[MINUTES_UNITS_ANODE] & ~ANODE_DATA[MINUTES_UNITS_ANODE]) :
    ANODE_DATA[MINUTES_UNITS_ANODE]
  );

  // minutes decimals display
  displaySegmentData(
    SEGMENT_DATA[minutesDecimalsDigitIndex],
    (blinkMinutes && blinkTicker) ?
    (ANODE_DATA[MINUTES_DECIMALS_ANODE] & ~ANODE_DATA[MINUTES_DECIMALS_ANODE]) :
    ANODE_DATA[MINUTES_DECIMALS_ANODE]
  );

  // hours units display
  displaySegmentData(
    secondsTick ?
    (SEGMENT_DATA[hoursUnitsDigitIndex] & SEGMENT_DATA[DISPLAY_DOT_INDEX]) :
    (SEGMENT_DATA[hoursUnitsDigitIndex] | ~SEGMENT_DATA[DISPLAY_DOT_INDEX]),
    (blinkHours && blinkTicker) ?
    (ANODE_DATA[HOURS_UNITS_ANODE] & ~ANODE_DATA[HOURS_UNITS_ANODE]) :
    ANODE_DATA[HOURS_UNITS_ANODE]
  );

  // hours decimals display
  displaySegmentData(
    SEGMENT_DATA[(hoursDecimalsDigitIndex != 0) ? hoursDecimalsDigitIndex : DISPLAY_BLANK_INDEX],
    (blinkHours && blinkTicker) ?
    (ANODE_DATA[HOURS_DECIMALS_ANODE] & ~ANODE_DATA[HOURS_DECIMALS_ANODE]) :
    ANODE_DATA[HOURS_DECIMALS_ANODE]
  );
}
// -----------------------------------------------------------------------------

// ------------------------- Sleep handling ------------------------------------
static void checkSleep(uint32_t sleepIntervalMs) {
  static uint32_t lastSleepTime;

  if(wakeUpFlag) {
    lastSleepTime = millis();
    wakeUpFlag = false;
  }

  if((millis() - lastSleepTime) >= sleepIntervalMs) {
    #ifdef DEBUG
    Serial.println(" -> going to sleep...");
    #endif
    blankDisplay();
    while(!wakeUpFlag) {
      // if no wakeup event then keep sleeping in a loop using Energia sleep API
      //  (LPM3 mode)
      sleep(SLEEP_INTERVAL_MS);
    }
    #ifdef DEBUG
    Serial.println(" -> waking up ...");
    #endif
    lastSleepTime = millis();
  }
}
// -----------------------------------------------------------------------------

// -------------------------- Keys handling ------------------------------------
static void keyChangeHandler() {
  wakeUpFlag = true;
}

static void checkKeysEvent() {
  if((digitalRead(MENU_KEY) == LOW) && (keyEvent.state != PRESSED)) {
    keyEvent.timestamp = millis();
    keyEvent.state = PRESSED;
  } else if((digitalRead(MENU_KEY) == HIGH) && (keyEvent.state != RELEASED)) {
    // calculate elapsed time since PRESS state
    uint32_t keyPressDuration = millis() - keyEvent.timestamp;

    #ifdef DEBUG
    Serial.print("duration: ");
    Serial.print(keyPressDuration);
    #endif
    // fire events based on time elapsed between key states
    if(keyPressDuration >= SHORT_KEYPRESS_MIN_THRESHOLD_MS &&
        keyPressDuration <= SHORT_KEYPRESS_MAX_THRESHOLD_MS) {
      #ifdef DEBUG
      Serial.println(" -> short keypress event");
      #endif

      keyEvent.event = SHORT_KEYPRESS_EVENT;
    }

    if(keyPressDuration >= LONG_KEYPRESS_MAX_THRESHOLD_MS) {
      #ifdef DEBUG
      Serial.println(" -> long keypress event");
      #endif

      keyEvent.event = LONG_KEYPRESS_EVENT;
    }

    // record last event
    lastKeyEvent = keyEvent.event;

    // reset current events state for a new iteration
    keyEvent.state = RELEASED;
    keyEvent.event = NO_KEYPRESS_EVENT;
    keyEvent.timestamp = 0;
  }
}
// -----------------------------------------------------------------------------

// ----------------------- FSM handling ----------------------------------------
static void checkStateMachine() {
  switch(nextState) {
    case DISPLAY_STATE:
      hoursBlink = false;
      minutesBlink = false;
      if(lastKeyEvent == LONG_KEYPRESS_EVENT) {
        nextState = MINUTES_SETUP_STATE;
        lastKeyEvent = NO_KEYPRESS_EVENT;
      }
    break;
    case MINUTES_SETUP_STATE:
      #ifdef DEBUG
      if(lastKeyEvent != NO_KEYPRESS_EVENT) {
        Serial.println(" -> fsm: minutes setup state");
      }
      #endif

      minutesBlink = true;
      hoursBlink = !minutesBlink;
      if(lastKeyEvent == SHORT_KEYPRESS_EVENT) {
        rtc.stop();
        rtc.Inc_min();
        currentTime.minutes = rtc.RTC_min;
        rtc.start();
        lastKeyEvent = NO_KEYPRESS_EVENT;

        #ifdef DEBUG
        Serial.print(" -> fsm: minutes increment value: ");
        Serial.println(currentTime.minutes);
        #endif
      }

      if(lastKeyEvent == LONG_KEYPRESS_EVENT) {
        nextState = HOURS_SETUP_STATE;
        lastKeyEvent = NO_KEYPRESS_EVENT;
      }
    break;
    case HOURS_SETUP_STATE:
      #ifdef DEBUG
      if(lastKeyEvent != NO_KEYPRESS_EVENT) {
        Serial.println(" -> fsm: hours setup state");
      }
      #endif

      hoursBlink = true;
      minutesBlink = !hoursBlink;
      if(lastKeyEvent == SHORT_KEYPRESS_EVENT) {
        rtc.stop();
        rtc.Inc_hr();
        currentTime.hours = rtc.RTC_hr;
        rtc.start();
        lastKeyEvent = NO_KEYPRESS_EVENT;

        #ifdef DEBUG
        Serial.print(" -> fsm: hours increment value: ");
        Serial.println(currentTime.hours);
        #endif
      }

      if(lastKeyEvent == LONG_KEYPRESS_EVENT) {
        nextState = DISPLAY_STATE;
        lastKeyEvent = NO_KEYPRESS_EVENT;
      }
    break;
  }
}
// -----------------------------------------------------------------------------

static void initDisplay() {
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(4);
  blankDisplay();
}

void setup() {
  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  // set all pins as output for low power
  #ifdef __MSP430_HAS_PORT1_R__
  P1DIR = 0xFF;
  #endif
  #ifdef __MSP430_HAS_PORT2_R__
  P2DIR = 0xFF;
  #endif

  // now set required pins mode as appropriate
  pinMode(DISPLAY_LATCH_PIN, OUTPUT);
  pinMode(MENU_KEY, INPUT_PULLUP);

  initDisplay();

  rtc.begin();

  attachInterrupt(MENU_KEY, keyChangeHandler, CHANGE);

  #ifdef DEBUG
  Serial.println("started...");
  #endif
}

void loop() {
  displayTime(getRtcTime(&rtc, &currentTime), minutesBlink, hoursBlink);
  checkKeysEvent();
  checkStateMachine();
  checkSleep(INACTIVITY_SLEEP_INTERVAL_MS);
}
