// Meera Al Khazraji & Vicor Nadu
// Professor Micheal Shiloh
// Performing Robots Fall '25

/*
   Using the nRF24L01 radio module to communicate
   between two Arduinos with much increased reliability following
   various tutorials, conversations, and studying the nRF24L01 datasheet
   and the library reference.

   Transmitter is
   https://github.com/michaelshiloh/resourcesForClasses/tree/master/kicad/Arduino_Shield_RC_Controller

  Receiver is
  https://github.com/michaelshiloh/resourcesForClasses/blob/master/kicad/nRF_servo_Mega

   This file contains code for both transmitter and receiver.
   Transmitter at the top, receiver at the bottom.
   One of them is commented out, so you need to comment in or out
   the correct section. You don't need to make changes to this 
   part of the code, just to comment in or out depending on
   whether you are programming your transmitter or receiver

   You need to set the correct address for your robot.

   Search for the phrase CHANGEHERE to see where to 
   comment or uncomment or make changes.

   These sketches require the RF24 library by TMRh20
   Documentation here: https://nrf24.github.io/RF24/index.html

   change log

   11 Oct 2023 - ms - initial entry based on
                  rf24PerformingRobotsTemplate
   26 Oct 2023 - ms - revised for new board: nRF_Servo_Mega rev 2
   28 Oct 2023 - ms - add demo of NeoMatrix, servo, and Music Maker Shield
	 20 Nov 2023 - as - fixed the bug which allowed counting beyond the limits
   22 Nov 2023 - ms - display radio custom address byte and channel
   12 Nov 2024 - ms - changed names for channel and address allocation for Fall 2024                  
                      https://github.com/michaelshiloh/resourcesForClasses/blob/master/kicad/nRF_servo_Mega    
                      https://github.com/michaelshiloh/resourcesForClasses/blob/master/kicad/nRFControlPanel
                      
   [USER] Nov 2025 - Integrated servo crank sweep functionality.
                     Simplified to single state.
                     Added non-blocking servo logic.
                     Added non-blocking transmitter logic.
                     Made crank positions easy to edit.
*/

// Common code
//

// Common pin usage
// Note there are additional pins unique to transmitter or receiver
//

// nRF24L01 uses SPI which is fixed
// on pins 11, 12, and 13 on the Uno
// and on pins 50, 51, and 52 on the Mega

// It also requires two other signals
// (CE = Chip Enable, CSN = Chip Select Not)
// Which can be any pins:

// CHANGEHERE
// For the transmitter
const int NRF_CE_PIN = A4, NRF_CSN_PIN = A5;

// CHANGEHERE
// for the receiver
//const int NRF_CE_PIN = A11, NRF_CSN_PIN = A15;

// nRF 24L01 pin   name
//          1      GND
//          2      3.3V
//          3      CE
//          4      CSN
//          5      SCLK
//          6      MOSI/COPI
//          7      MISO/CIPO

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);  // CE, CSN

//#include <printf.h>  // for debugging

// See note in rf24Handshaking about address selection
//

// Channel and address allocation:
// Torico and Sarah: Channel 30, addr = 0x76
// Sudiksha and Aysha: Channel 40, addr = 0x73
// Mariam and Joy:  Channel 50, addr = 0x7C
// Ghadir and Mustafa: Channel 60, addr = 0xC6
// Clara and Jiho:  Channel 70, addr = 0xC3
// Victor and Meera: Channel 80, addr = 0xCC
// Ali and Hari: Channel 90, addr = 0x33

// CHANGEHERE
const int CUSTOM_CHANNEL_NUMBER = 80;   // change as per the above assignment
const byte CUSTOM_ADDRESS_BYTE = 0xCC;  // change as per the above assignment

// Do not make changes here
const byte xmtrAddress[] = { CUSTOM_ADDRESS_BYTE, CUSTOM_ADDRESS_BYTE, 0xC7, 0xE6, 0xCC };
const byte rcvrAddress[] = { CUSTOM_ADDRESS_BYTE, CUSTOM_ADDRESS_BYTE, 0xC7, 0xE6, 0x66 };

const int RF24_POWER_LEVEL = RF24_PA_LOW;

// global variables
uint8_t pipeNum;
unsigned int totalTransmitFailures = 0;

struct DataStruct {
  uint8_t stateNumber;
};
DataStruct data;

void setupRF24Common() {

  // RF24 setup
  if (!radio.begin()) {
    Serial.println(F("radio  initialization failed"));
    while (1)
      ;
  } else {
    Serial.println(F("radio successfully initialized"));
  }

  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(CUSTOM_CHANNEL_NUMBER);
  radio.setPALevel(RF24_POWER_LEVEL);
}


// CHANGEHERE
// Transmitter code

// Transmitter pin usage
const int LCD_RS_PIN = 3, LCD_EN_PIN = 2, LCD_D4_PIN = 4, LCD_D5_PIN = 5, LCD_D6_PIN = 6, LCD_D7_PIN = 7;
const int SW1_PIN = 8, SW2_PIN = 9, SW3_PIN = 10, SW4_PIN = A3, SW5_PIN = A2;

// LCD library code
#include <LiquidCrystal.h>

// initialize the library with the relevant pins
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);


// MEERA EDIT
const int NUM_OF_STATES = 38;
char* theStates[] = {
  // 0–8  : Scene 1 – Opening Aside (Tracks 001–009)
  "Well, that didn't",        // 0 - Track001
  "Faithful as a coin",       // 1 - Track002
  "Over there, the wife",     // 2 - Track003
  "Paternity is a jackpot",   // 3 - Track004
  "Flexible in every sense",  // 4 - Track005
  "Saulbot or something",     // 5 - Track006
  "Presiding is our judge",   // 6 - Track007
  "This one's going to be",   // 7 - Track008
  "Tension, deposit fees",    // 8 - Track009
  // 9–17 : Scene 3 – Banter (Tracks 010–018)
  "I walk with confidence",     // 9  - Track010
  "Drama like a side",          // 10 - Track011
  "Slot machines communicate",  // 11 - Track012
  "Plays dignity like coupon",  // 12 - Track013
  "Play the house better",      // 13 - Track014
  "Only sane ones here",        // 14 - Track015
  "So what's your angle",       // 15 - Track016
  "Consistently pathetic",      // 16 - Track017
  "I think they mean",          // 17 - Track018
  // 18–19 : Scene 3 – Beat with Child / Resolve (Tracks 019–020)
  "We need to win",           // 18 - Track019
  "I'll save that pleasure",  // 19 - Track020
  // 20–27 : Scene 5 – Court Monologue (Tracks 021–028)
  "Casanova with rap sheet",   // 20 - Track021
  "Sure, he danced",           // 21 - Track022
  "That house like Vegas",     // 22 - Track023
  "Saulbot, downfall of civ",  // 23 - Track024
  "Preacher on Sunday",        // 24 - Track025
  "Harassment claims A to Z",  // 25 - Track026
  "I trust your judgment",     // 26 - Track027
  "Same damn thing",           // 27 - Track028
  // 28–33 : Child Questioning (Tracks 029–034)
  "Now, tell us kid",           // 28 - Track029
  "Betraying or just dancing",  // 29 - Track030
  "What was father thinking",   // 30 - Track031
  "Kids say darndest things",   // 31 - Track032
  "Who's the better parent",    // 32 - Track033
  "Between mother and father",  // 33 - Track034
  // 34–35 : Jackpot / Panic (Tracks 035–036)
  "JACKPOT!",             // 34 - Track035
  "Most lucrative case",  // 35 - Track036
  // 36 : Listening mode (no audio just neos)
  "LISTENING mode",  // 36 - SW4 state
  "IDLE mode",       // 37 - SW5 state (new idle/default)

};


void updateLCD() {
  lcd.clear();
  lcd.print(theStates[data.stateNumber]);
  lcd.setCursor(0, 1);  // column, line (from 0)
  lcd.print("not transmitted yet");
}

void countDown() {
  data.stateNumber = (data.stateNumber > 0) ? (data.stateNumber - 1) : 0;
  updateLCD();
}

void countUp() {
  if (++data.stateNumber >= NUM_OF_STATES) {
    data.stateNumber = NUM_OF_STATES - 1;
  }
  updateLCD();
}

void spare1() {
  data.stateNumber = 36;  // jump to LISTENING mode
  updateLCD();            // show it on the screen
  rf24SendData();
}
void spare2() {
  data.stateNumber = 37;  // jump to IDLE mode
  updateLCD();
  rf24SendData();
}

void rf24SendData() {

  radio.stopListening();  // go into transmit mode
  // The write() function will block
  // until the message is successfully acknowledged by the receiver
  // or the timeout/retransmit maxima are reached.
  // Returns 1 if write succeeds
  // Returns 0 if errors occurred (timeout or FAILURE_HANDLING fails)
  int retval = radio.write(&data, sizeof(data));

  lcd.clear();
  lcd.setCursor(0, 0);  // column, line (from 0)
  lcd.print("transmitting");
  lcd.setCursor(14, 0);  // column, line (from 0)
  lcd.print(data.stateNumber);

  Serial.print(F(" ... "));
  if (retval) {
    Serial.println(F("success"));
    lcd.setCursor(0, 1);  // column, line (from 0)
    lcd.print("success");
  } else {
    totalTransmitFailures++;
    Serial.print(F("failure, total failures = "));
    Serial.println(totalTransmitFailures);

    lcd.setCursor(0, 1);  // column, line (from 0)
    lcd.print("error, total=");
    lcd.setCursor(13, 1);  // column, line (from 0)
    lcd.print(totalTransmitFailures);
  }
}

class Button {
  int pinNumber;
  bool previousState;
  void (*buttonFunction)();
public:

  // Constructor
  Button(int pn, void* bf) {
    pinNumber = pn;
    buttonFunction = bf;
    previousState = 1;
  }

  // update the button
  void update() {
    bool currentState = digitalRead(pinNumber);
    if (currentState == LOW && previousState == HIGH) {
      Serial.print("button on pin ");
      Serial.print(pinNumber);
      Serial.println();
      buttonFunction();
    }
    previousState = currentState;
  }
};

const int NUMBUTTONS = 5;
Button theButtons[] = {
  Button(SW1_PIN, countDown),
  Button(SW2_PIN, rf24SendData),
  Button(SW3_PIN, countUp),
  Button(SW4_PIN, spare1),
  Button(SW5_PIN, spare2),
};

void setupRF24() {

  // Check whether the correct pins are assigned
  if (NRF_CE_PIN != A4 || NRF_CSN_PIN != A5) {
    Serial.println(F("The wrong NRF_CE_PIN and NRF_CSN_PIN pins are defined for a transmitter"));
    while (1)
      ;
  }

  setupRF24Common();

  // Set us as a transmitter
  radio.openWritingPipe(xmtrAddress);
  radio.openReadingPipe(1, rcvrAddress);

  // radio.printPrettyDetails();
  Serial.println(F("I am a transmitter"));

  data.stateNumber = 0;
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("Setting up LCD"));

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  // Print a message to the LCD.
  lcd.print("Radio setup");

  // Display the address in hex
  lcd.setCursor(0, 1);
  lcd.print("addr 0x");
  lcd.setCursor(7, 1);
  char s[5];
  sprintf(s, "%02x", CUSTOM_ADDRESS_BYTE);
  lcd.print(s);

  // Display the channel number
  lcd.setCursor(10, 1);
  lcd.print("ch");
  lcd.setCursor(13, 1);
  lcd.print(CUSTOM_CHANNEL_NUMBER);

  Serial.println(F("Setting up radio"));
  setupRF24();

  // If setupRF24 returned then the radio is set up
  lcd.setCursor(0, 0);
  lcd.print("Radio OK state=");
  lcd.print(theStates[data.stateNumber]);

  // Initialize the switches
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);
  pinMode(SW3_PIN, INPUT_PULLUP);
  pinMode(SW4_PIN, INPUT_PULLUP);
  pinMode(SW5_PIN, INPUT_PULLUP);
}



void loop() {
  for (int i = 0; i < NUMBUTTONS; i++) {
    theButtons[i].update();
  }
  delay(50);  // for testing
}


void clearData() {
  // set all fields to 0
  data.stateNumber = 0;
}

// End of transmitter code
// CHANGEHERE


/*

// Receiver Code
// CHANGEHERE


// Uncomment this to activate the receiver code
// Additional libraries for music maker shield
#include <Adafruit_VS1053.h>
#include <SD.h>

// Servo library
#include <Servo.h>

// Additional libraries for graphics on the Neo Pixel Matrix
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#ifndef PSTR
#define PSTR  // Make Arduino Due happy
#endif


// *** MODIFICATION: Add easy-to-edit variables for crank tuning ***
// *** MODIFICATION: Renamed servo pin **
// This is  "up" or "resting" position. 0 is highest, 180 is lowest.
const int CRANK_REST_POSITION = 50;
// This is "down" or "pulled" position.
const int CRANK_PULL_POSITION = 130;
// This controls the speed. Smaller number = faster.
const int crankMoveDelay = 15;   // ms between crank steps
const int CRANK_SERVO_PIN = 20;  // This is the pin for your crank servo

// MEERA EDITS
const int NEOPIXELPIN = 17;
const int NUMPIXELS = 31;
const int MOTOR_PIN = 8;        // DC motor control pin
const int MOTOR_OFF = 0;        // Stopped
const int MOTOR_IDLE = 100;     // Slow, calm ~40% speed
const int MOTOR_NORMAL = 150;   // Regular speed ~60%
const int MOTOR_SLEAZY = 180;   // Medium-fast ~70%
const int MOTOR_EXCITED = 220;  // Fast, energetic ~85%
const int MOTOR_ANGRY = 255;    // Maximum speed 100%
bool isFlashingRed = false;
unsigned long redFlashStartTime = 0;
const unsigned long RED_FLASH_DURATION = 4500;
bool isFlashingRainbow = false;
bool isBreathing = false;
bool isPulsing = false;
bool isSparkle = false;
bool isWaving = false;
bool isStrobing = false;
bool isAlternating = false;
unsigned long lastWaveUpdate = 0;
int wavePosition = 0;
uint8_t wave_r1 = 0, wave_g1 = 0, wave_b1 = 0;
uint8_t wave_r2 = 0, wave_g2 = 0, wave_b2 = 0;
unsigned long lastEffectUpdate = 0;
int breathingBrightness = 0;
int breathingDirection = 1;
unsigned long rainbowFlashStartTime = 0;
const unsigned long RAINBOW_FLASH_DURATION = 48000;
unsigned long lastRainbowUpdate = 0;
const unsigned long RAINBOW_UPDATE_DELAY = 20;  // Update every 20ms for smooth animation
uint16_t rainbowOffset = 0;
#define NEOPIN 17
#define N_LEDS 32
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, NEOPIN, NEO_GRB + NEO_KHZ800);
// Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8, 8, NEOPIXELPIN,
//                             NEO_MATRIX_TOP     + NEO_MATRIX_RIGHT +
//                             NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
//                             NEO_GRB            + NEO_KHZ800);

// Adafruit music maker shield
#define SHIELD_RESET -1  // VS1053 reset pin (unused!)
#define SHIELD_CS 7      // VS1053 chip select pin (output)
#define SHIELD_DCS 6     // VS1053 Data/command select pin (output)
#define CARDCS 4         // Card chip select pin \
                         // // DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3           // VS1053 Data request, ideally an Interrupt pin
Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);

// Connectors for NeoPixels and Servo Motors are labeled
// M1 - M6 which is not very useful. Here are the pin
// assignments:
// M1 = 19
// M2 = 20
// M3 = 21
// M4 = 16
// M5 = 18
// M6 = 17

// Servo motors
// *** MODIFICATION: Renamed servo object ***
Servo crank;  // This servo object will control your crank on pin 20


// *** MODIFICATION: Add variables for non-blocking crank pull ***
bool isCrankPulling = false;                  // True if the pull is in progress
bool isReturningToRest = false;               // True if on the return trip
int currentCrankAngle = CRANK_REST_POSITION;  // Current angle of the crank
int crankPullDirection = 1;                   // Direction of pull (1 = forward, -1 = reverse)
unsigned long lastCrankMoveTime = 0;          // Timestamp of the last crank move

// MEERA EDIT
void setup() {
  Serial.begin(9600);
  // --- DC motor simple spin in setup ---
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, HIGH);  // turn motor ON
  delay(5000);                    // keep it on for 5 seconds
  digitalWrite(MOTOR_PIN, LOW);   // turn motor OFF
  // printf_begin();
  // Set up all the attached hardware
  setupMusicMakerShield();
  setupServoMotors();
  //setupNeoPixels();
  strip.begin();
  strip.show();  // Initialize all pixels to 'off'
  setupRF24();
  // Brief flash to show we're done with setup()
  flashNeoPixels();
}

void setupRF24() {
  setupRF24Common();

  // Set us as a receiver
  radio.openWritingPipe(rcvrAddress);
  radio.openReadingPipe(1, xmtrAddress);

  // radio.printPrettyDetails();
  Serial.println(F("I am a receiver"));
}

void setupMusicMakerShield() {
  if (!musicPlayer.begin()) {  // initialise the music player
    Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
    while (1)
      ;
  }
  Serial.println(F("VS1053 found"));

  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD card failed or not present"));
    while (1)
      ;  // don't do anything more
  }

  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(20, 20);

  // Timer interrupts are not suggested, better to use DREQ interrupt!
  //musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT); // timer int

  // If DREQ is on an interrupt pin (on uno, #2 or #3) we can do background
  // audio playing
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
}

void setupServoMotors() {
  crank.attach(CRANK_SERVO_PIN);     // Attaches the servo on pin 20
  crank.write(CRANK_REST_POSITION);  // Sets initial position
}


// void setupNeoPixels() {
//   //  pixels.begin();
//   //  pixels.clear();
//   //  pixels.show();
//   matrix.begin();
//   matrix.setTextWrap(false);
//   matrix.setBrightness(40);
//   matrix.setTextColor(matrix.Color(200, 30, 40));
// }


// MEERA EDIT helper funcs for neopix
void setColor(uint8_t r, uint8_t g, uint8_t b) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(g, r, b));  // Swapped R and G as this strip is grb not rgb
  }
  strip.show();
}

void updateRedFlash() {
  if (!isFlashingRed) {
    return;
  }

  if (millis() - redFlashStartTime >= RED_FLASH_DURATION) {
    setColor(0, 0, 0);  // Turn off
    isFlashingRed = false;
    Serial.println(F("Red flash finished."));
  }
}

void updateRainbowEffect() {
  if (!isFlashingRainbow) {
    return;  // Not active, exit
  }
  // Check if 5 seconds have passed
  if (millis() - rainbowFlashStartTime >= RAINBOW_FLASH_DURATION) {
    setColor(0, 0, 0);  // Turn off
    isFlashingRainbow = false;
    Serial.println(F("Rainbow effect finished."));
    return;
  }
  // Update rainbow colors every RAINBOW_UPDATE_DELAY milliseconds
  if (millis() - lastRainbowUpdate >= RAINBOW_UPDATE_DELAY) {
    lastRainbowUpdate = millis();

    // Update each LED with rainbow colors
    for (uint16_t i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + rainbowOffset) & 255));
    }
    strip.show();
    rainbowOffset++;  // Cycle the rainbow
    if (rainbowOffset >= 256) rainbowOffset = 0;
  }
}

// MEERA EDIT
void flashNeoPixels() {
  setColor(255, 255, 255);  // Turn all white
  delay(1000);
  setColor(0, 0, 0);  // Turn all OFF
  strip.clear();
  strip.show();
  delay(100);
}

// Helper function for breathing effect
void updateBreathing(uint8_t r, uint8_t g, uint8_t b) {
  if (!isBreathing) return;

  if (millis() - lastEffectUpdate >= 20) {
    lastEffectUpdate = millis();

    breathingBrightness += breathingDirection * 5;
    if (breathingBrightness >= 255) {
      breathingBrightness = 255;
      breathingDirection = -1;
    }
    if (breathingBrightness <= 0) {
      breathingBrightness = 0;
      breathingDirection = 1;
    }

    // Scale colors by brightness
    uint8_t r_scaled = (r * breathingBrightness) / 255;
    uint8_t g_scaled = (g * breathingBrightness) / 255;
    uint8_t b_scaled = (b * breathingBrightness) / 255;
    setColor(r_scaled, g_scaled, b_scaled);
  }
}

// Helper function for pulsing effect
void updatePulsing(uint8_t r, uint8_t g, uint8_t b) {
  if (!isPulsing) return;

  if (millis() - lastEffectUpdate >= 100) {
    lastEffectUpdate = millis();

    breathingBrightness += breathingDirection * 30;
    if (breathingBrightness >= 255) {
      breathingBrightness = 255;
      breathingDirection = -1;
    }
    if (breathingBrightness <= 100) {
      breathingBrightness = 100;
      breathingDirection = 1;
    }

    uint8_t r_scaled = (r * breathingBrightness) / 255;
    uint8_t g_scaled = (g * breathingBrightness) / 255;
    uint8_t b_scaled = (b * breathingBrightness) / 255;
    setColor(r_scaled, g_scaled, b_scaled);
  }
}

// Helper function for sparkle effect
void updateSparkle(uint8_t r, uint8_t g, uint8_t b) {
  if (!isSparkle) return;

  if (millis() - lastEffectUpdate >= 50) {
    lastEffectUpdate = millis();

    // Random pixels flicker
    for (int i = 0; i < strip.numPixels(); i++) {
      if (random(10) > 7) {  // 30% chance
        strip.setPixelColor(i, strip.Color(g, r, b));
      } else {
        strip.setPixelColor(i, strip.Color(0, 0, 0));
      }
    }
    strip.show();
  }
}

// Helper function for wave effect (flows across strip)
void updateWave(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2) {
  if (!isWaving) return;

  if (millis() - lastWaveUpdate >= 50) {
    lastWaveUpdate = millis();

    for (int i = 0; i < strip.numPixels(); i++) {
      int offset = (i + wavePosition) % strip.numPixels();
      if (offset < strip.numPixels() / 2) {
        strip.setPixelColor(i, strip.Color(g1, r1, b1));  // GRB
      } else {
        strip.setPixelColor(i, strip.Color(g2, r2, b2));  // GRB
      }
    }
    strip.show();
    wavePosition = (wavePosition + 1) % strip.numPixels();
  }
}

// Helper function for strobing effect
void updateStrobe(uint8_t r, uint8_t g, uint8_t b) {
  if (!isStrobing) return;

  if (millis() - lastEffectUpdate >= 150) {
    lastEffectUpdate = millis();

    static bool strobeOn = false;
    strobeOn = !strobeOn;

    if (strobeOn) {
      setColor(r, g, b);
    } else {
      setColor(0, 0, 0);
    }
  }
}

// Motor helper funcs
void setMotorSpeed(int speed) {
  analogWrite(MOTOR_PIN, speed);
}

void stopMotor() {
  analogWrite(MOTOR_PIN, 0);
}

// Helper function for alternating effect
void updateAlternating(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2) {
  if (!isAlternating) return;

  if (millis() - lastEffectUpdate >= 300) {
    lastEffectUpdate = millis();

    static bool toggle = false;
    toggle = !toggle;

    for (int i = 0; i < strip.numPixels(); i++) {
      if ((i % 2 == 0) == toggle) {
        strip.setPixelColor(i, strip.Color(g1, r1, b1));  // GRB
      } else {
        strip.setPixelColor(i, strip.Color(g2, r2, b2));  // GRB
      }
    }
    strip.show();
  }
}
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    // Return color in GRB order for your strip
    return strip.Color(0, 255 - WheelPos * 3, WheelPos * 3);  // G, R, B
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(WheelPos * 3, 0, 255 - WheelPos * 3);  // G, R, B
  }
  WheelPos -= 170;
  return strip.Color(255 - WheelPos * 3, WheelPos * 3, 0);  // G, R, B
}

// meera edit
// *** MODIFICATION: Fixed logic in updateCrankPull ***
// void updateCrankPull() {
//   // Only run this code if the crank pull is active
//   if (!isCrankPulling) {
//     return;
//   }

void updateCrankPull() {
  if (!isCrankPulling) return;

  if (millis() - lastCrankMoveTime >= crankMoveDelay) {
    lastCrankMoveTime = millis();

    // Move one step in the current direction
    currentCrankAngle += crankPullDirection;
    crank.write(currentCrankAngle);

    if (!isReturningToRest) {
      // FIRST LEG: REST -> PULL
      if ((crankPullDirection > 0 && currentCrankAngle >= CRANK_PULL_POSITION) || (crankPullDirection < 0 && currentCrankAngle <= CRANK_PULL_POSITION)) {

        currentCrankAngle = CRANK_PULL_POSITION;
        crank.write(currentCrankAngle);

        crankPullDirection *= -1;  // reverse
        isReturningToRest = true;
      }
    } else {
      // SECOND LEG: PULL -> REST
      if ((crankPullDirection > 0 && currentCrankAngle >= CRANK_REST_POSITION) || (crankPullDirection < 0 && currentCrankAngle <= CRANK_REST_POSITION)) {

        currentCrankAngle = CRANK_REST_POSITION;
        crank.write(currentCrankAngle);

        isCrankPulling = false;
        isReturningToRest = false;
        Serial.println(F("Crank pull finished. Detaching servo."));
        crank.detach();  // <<< IMPORTANT
      }
    }
  }
}

// MEERA EDIT
void loop() {
  updateCrankPull();
  updateRedFlash();
  updateRainbowEffect();
  updateBreathing(128, 0, 128);
  updatePulsing(255, 0, 100);
  updateSparkle(255, 200, 0);
  updateWave(wave_r1, wave_g1, wave_b1, wave_r2, wave_g2, wave_b2);
  updateStrobe(255, 0, 0);
  updateAlternating(255, 200, 0, 255, 100, 0);  // Yellow/Orange for listening

  radio.startListening();
  if (radio.available(&pipeNum)) {
    radio.read(&data, sizeof(data));
    Serial.print(F("message received Data = "));
    Serial.print(data.stateNumber);
    Serial.println();

    // Reset ALL effects
    isBreathing = false;
    isPulsing = false;
    isSparkle = false;
    isFlashingRed = false;
    isFlashingRainbow = false;
    isWaving = false;
    isStrobing = false;
    isAlternating = false;

    switch (data.stateNumber) {

        // ===== SCENE 1 =====

      case 0:  // Track001 - "Well, that didn't take long..."
        Serial.println(F("Dark Purple Breathing"));
        if (crank.attached()) {
          crank.write(CRANK_REST_POSITION);
          crank.detach();
        }
        isCrankPulling = false;
        isBreathing = true;
        breathingBrightness = 0;
        breathingDirection = 1;
        setMotorSpeed(MOTOR_IDLE);  // Slow entrance
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track001.mp3");
        break;

      case 1:  // Track002 - "Faithful as a coin toss"
        Serial.println(F("Gold Sparkle"));
        if (crank.attached()) crank.detach();
        isSparkle = true;
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track002.mp3");
        break;

      case 2:                                               // Track003 - "Wife holds a smile"
        Serial.println(F("White to Cold Blue Breathing"));  // UPGRADED!
        if (crank.attached()) crank.detach();
        setColor(255, 255, 255);  // White flash
        delay(300);
        isBreathing = true;
        breathingBrightness = 128;
        breathingDirection = -1;
        setMotorSpeed(MOTOR_IDLE);  // Cold, slow
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track003.mp3");
        break;

      case 3:  // Track004 - "Paternity jackpot"
        Serial.println(F("Dark Red Pulse → Gold Flash + PULL"));
        isPulsing = true;
        breathingBrightness = 100;
        breathingDirection = 1;
        setMotorSpeed(MOTOR_EXCITED);  // Big reveal energy!
        delay(1000);
        setColor(255, 200, 0);  // Gold flash
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        crankPullDirection = (CRANK_REST_POSITION < CRANK_PULL_POSITION) ? 1 : -1;
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track004.mp3");
        break;

      case 4:  // Track005 - "Flexible in every sense"
        Serial.println(F("Hot Pink Fast Pulse"));
        if (crank.attached()) crank.detach();
        isPulsing = true;
        breathingBrightness = 255;
        breathingDirection = -1;
        setMotorSpeed(MOTOR_SLEAZY);  // Sleazy speed!
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track005.mp3");
        break;

      case 5:  // Track006 - "Saulbot or something"
        Serial.println(F("Toxic Green Wave"));
        if (crank.attached()) crank.detach();
        isWaving = true;
        wave_r1 = 0;
        wave_g1 = 255;
        wave_b1 = 0;
        wave_r2 = 0;
        wave_g2 = 100;
        wave_b2 = 0;
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track006.mp3");
        break;

      case 6:  // Track007 - "Our judge"
        Serial.println(F("Royal Blue Breathing"));
        if (crank.attached()) crank.detach();
        isBreathing = true;
        breathingBrightness = 0;
        breathingDirection = 1;
        setMotorSpeed(MOTOR_IDLE);  // Respectful, calm
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track007.mp3");
        break;

      case 7:  // Track008 - "This one's going to be fun"
        Serial.println(F("RAINBOW CHASE + PULL"));
        isFlashingRainbow = true;
        rainbowFlashStartTime = millis();
        lastRainbowUpdate = millis();
        rainbowOffset = 0;
        setMotorSpeed(MOTOR_EXCITED);  // Pumped up!
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        crankPullDirection = (CRANK_REST_POSITION < CRANK_PULL_POSITION) ? 1 : -1;
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track008.mp3");
        break;

      case 8:  // Track009 - "Deposit fees"
        Serial.println(F("Gold Sparkle Intense"));
        if (crank.attached()) crank.detach();
        isSparkle = true;
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track009.mp3");
        break;

        // SCENE 3

      case 9:  // Track010 - "I walk with confidence"
        Serial.println(F("Purple Strut Wave"));
        if (crank.attached()) crank.detach();
        isWaving = true;
        wave_r1 = 150;
        wave_g1 = 0;
        wave_b1 = 150;
        wave_r2 = 50;
        wave_g2 = 0;
        wave_b2 = 50;
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track010.mp3");
        break;

      case 10:  // Track011 - "Drama side hustle"
        Serial.println(F("Spinning Orange/Red"));
        if (crank.attached()) crank.detach();
        isWaving = true;
        wave_r1 = 255;
        wave_g1 = 100;
        wave_b1 = 0;
        wave_r2 = 255;
        wave_g2 = 0;
        wave_b2 = 0;
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track011.mp3");
        break;

      case 11:  // Track012 - "Slot machines"
        Serial.println(F("Slot Machine Effect"));
        if (crank.attached()) crank.detach();
        setMotorSpeed(MOTOR_EXCITED);  // Casino energy!
        for (int i = 0; i < 5; i++) {
          setColor(random(255), random(255), random(255));
          delay(100);
        }
        setColor(255, 200, 0);  // GOLD jackpot
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track012.mp3");
        break;

      case 12:                                             // Track013 - "Coupon dignity"
        Serial.println(F("Pink Sparkle to Trash Brown"));  // UPGRADED!
        if (crank.attached()) crank.detach();
        isSparkle = true;
        delay(800);
        setColor(100, 50, 0);  // Trash brown
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track013.mp3");
        break;

      case 13:  // Track014 - "Play the house"
        Serial.println(F("Casino Roulette"));
        if (crank.attached()) crank.detach();
        isAlternating = true;
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track014.mp3");
        break;

      case 14:  // Track015 - "Only sane ones"
        Serial.println(F("Calm Teal Breathing"));
        if (crank.attached()) crank.detach();
        isBreathing = true;
        breathingBrightness = 0;
        breathingDirection = 1;
        setMotorSpeed(MOTOR_IDLE);  // Fake calm
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track015.mp3");
        break;

      case 15:  // Track016 - "What's your angle"
        Serial.println(F("Sarcastic Rainbow"));
        if (crank.attached()) crank.detach();
        isFlashingRainbow = true;
        rainbowFlashStartTime = millis();
        lastRainbowUpdate = millis();
        rainbowOffset = 0;
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track016.mp3");
        break;

      case 16:                                       // Track017 - "Consistently pathetic"
        Serial.println(F("Blue to Sad Grey Wave"));  // UPGRADED!
        if (crank.attached()) crank.detach();
        isWaving = true;
        wave_r1 = 0;
        wave_g1 = 0;
        wave_b1 = 200;  // Blue
        wave_r2 = 100;
        wave_g2 = 100;
        wave_b2 = 100;  // Grey
        setMotorSpeed(MOTOR_IDLE);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track017.mp3");
        break;

      case 17:                                       // Track018 - "I think they mean us"
        Serial.println(F("Spotlight White Pulse"));  // UPGRADED!
        if (crank.attached()) crank.detach();
        isPulsing = true;
        breathingBrightness = 255;
        breathingDirection = -1;
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track018.mp3");
        break;

      case 18:  // Track019 - "We need to win"
        Serial.println(F("Determined Red Pulse"));
        if (crank.attached()) crank.detach();
        isPulsing = true;
        breathingBrightness = 255;
        breathingDirection = -1;
        setMotorSpeed(MOTOR_ANGRY);  // Intense!
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track019.mp3");
        break;

      case 19:                                       // Track020 - "Save that pleasure"
        Serial.println(F("Smug Purple Breathing"));  // UPGRADED!
        if (crank.attached()) crank.detach();
        isBreathing = true;
        breathingBrightness = 200;
        breathingDirection = 1;
        setMotorSpeed(MOTOR_SLEAZY);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track020.mp3");
        break;

        // SCENE 5 - COURTROOM

      case 20:  // Track021 - "Casanova rap sheet"
        Serial.println(F("WHITE FLASH → PURPLE + PULL"));
        setColor(255, 255, 255);
        delay(200);
        isPulsing = true;
        breathingBrightness = 150;
        breathingDirection = 1;
        setMotorSpeed(MOTOR_EXCITED);  // Grand opening!
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        crankPullDirection = (CRANK_REST_POSITION < CRANK_PULL_POSITION) ? 1 : -1;
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track021.mp3");
        break;

      case 21:  // Track022 - "Sure he danced"
        Serial.println(F("Lazy Orange Sway"));
        if (crank.attached()) crank.detach();
        isWaving = true;
        wave_r1 = 255;
        wave_g1 = 150;
        wave_b1 = 0;
        wave_r2 = 200;
        wave_g2 = 100;
        wave_b2 = 0;
        setMotorSpeed(MOTOR_IDLE);  // Dismissive, lazy
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track022.mp3");
        break;

      case 22:  // Track023 - "Vegas casino"
        Serial.println(F("Vegas Neon Strobe"));
        if (crank.attached()) crank.detach();
        setMotorSpeed(MOTOR_EXCITED);  // Vegas energy!
        for (int i = 0; i < 3; i++) {
          setColor(255, 200, 0);
          delay(100);
          setColor(255, 0, 0);
          delay(100);
          setColor(255, 100, 100);
          delay(100);
        }
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track023.mp3");
        break;

      case 23:  // Track024 - "Saulbot downfall"
        Serial.println(F("Apocalyptic Red Flash"));
        if (crank.attached()) crank.detach();
        isStrobing = true;
        setMotorSpeed(MOTOR_ANGRY);  // Aggressive mockery!
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track024.mp3");
        break;

      case 24:                                                         // Track025 - "Preacher/bachelor"
        Serial.println(F("Holy White Breathing → Party Pink Pulse"));  // UPGRADED!
        if (crank.attached()) crank.detach();
        isBreathing = true;
        breathingBrightness = 255;
        breathingDirection = -1;
        delay(900);
        isPulsing = true;
        setMotorSpeed(MOTOR_SLEAZY);  // Hypocrisy!
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track025.mp3");
        break;

      case 25:                                           // Track026 - "Harassment alphabetically"
        Serial.println(F("SCANDAL RED STROBE + PULL"));  // UPGRADED!
        isStrobing = true;
        setMotorSpeed(MOTOR_ANGRY);  // Maximum scandal energy!
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        crankPullDirection = (CRANK_REST_POSITION < CRANK_PULL_POSITION) ? 1 : -1;
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track026.mp3");
        break;

      case 26:  // Track027 - "I trust your judgment"
        Serial.println(F("Flattering Blue/Gold Wave"));
        if (crank.attached()) crank.detach();
        isWaving = true;
        wave_r1 = 0;
        wave_g1 = 0;
        wave_b1 = 200;
        wave_r2 = 255;
        wave_g2 = 200;
        wave_b2 = 0;
        setMotorSpeed(MOTOR_IDLE);  // Smooth flattery
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track027.mp3");
        break;

      case 27:  // Track028 - "Same damn thing"
        Serial.println(F("Chaotic Multi-Color Flicker"));
        if (crank.attached()) crank.detach();
        isSparkle = true;
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track028.mp3");
        break;

        // CHILD QUESTIONING

      case 28:  // Track029 - "Tell us kid"
        Serial.println(F("Interrogation Yellow Spotlight"));
        if (crank.attached()) crank.detach();
        isPulsing = true;  // UPGRADED from solid!
        breathingBrightness = 255;
        breathingDirection = -1;
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track029.mp3");
        break;

      case 29:                                       // Track030 - "Betraying or dancing"
        Serial.println(F("Gentle Blue Breathing"));  // UPGRADED!
        if (crank.attached()) crank.detach();
        isBreathing = true;
        breathingBrightness = 128;
        breathingDirection = 1;
        setMotorSpeed(MOTOR_IDLE);  // Gentle manipulation
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track030.mp3");
        break;

      case 30:  // Track031 - "What was father thinking"
        Serial.println(F("Teal Probing Pulse"));
        if (crank.attached()) crank.detach();
        isPulsing = true;
        breathingBrightness = 200;
        breathingDirection = 1;
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track031.mp3");
        break;

      case 31:  // Track032 - "Darndest things"
        Serial.println(F("Nervous Orange Flicker"));
        if (crank.attached()) crank.detach();
        isSparkle = true;
        setMotorSpeed(MOTOR_EXCITED);  // Nervous energy!
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track032.mp3");
        break;

      case 32:                                // Track033 - "Better parent"
        Serial.println(F("Trap Red Pulse"));  // UPGRADED!
        if (crank.attached()) crank.detach();
        isPulsing = true;
        breathingBrightness = 150;
        breathingDirection = 1;
        setMotorSpeed(MOTOR_NORMAL);
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track033.mp3");
        break;

      case 33:  // Track034 - "Between mother and father"
        Serial.println(F("Panic Yellow/White Flicker"));
        if (crank.attached()) crank.detach();
        for (int i = 0; i < strip.numPixels(); i++) {
          if (random(2)) {
            strip.setPixelColor(i, strip.Color(255, 255, 255));
          } else {
            strip.setPixelColor(i, strip.Color(255, 255, 0));
          }
        }
        strip.show();
        setMotorSpeed(MOTOR_EXCITED);  // Panic!
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track034.mp3");
        break;

        // VICTORY & PANIC

      case 34:  // Track035 - "JACKPOT!"
        Serial.println(F("EXPLOSIVE RAINBOW + MAX PULL"));
        setColor(255, 255, 255);
        delay(100);
        setColor(0, 0, 0);
        delay(50);
        setColor(255, 255, 255);
        delay(100);
        isFlashingRainbow = true;
        rainbowFlashStartTime = millis();
        lastRainbowUpdate = millis();
        rainbowOffset = 0;
        setMotorSpeed(MOTOR_ANGRY);  // MAXIMUM ENERGY!
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        crankPullDirection = (CRANK_REST_POSITION < CRANK_PULL_POSITION) ? 1 : -1;
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track035.mp3");
        break;

      case 35:  // Track036 - "Most lucrative case"
        Serial.println(F("Desperate Chaos Strobe"));
        if (crank.attached()) crank.detach();
        isStrobing = true;
        setMotorSpeed(MOTOR_ANGRY);  // Panicking!
        if (musicPlayer.playingMusic) musicPlayer.stopPlaying();
        musicPlayer.startPlayingFile("/track036.mp3");
        break;

        // 2 Idle modes

      case 36:  // LISTENING MODE (SW4)
        Serial.println(F("LISTENING: Yellow/Orange Alternating"));
        if (crank.attached()) crank.detach();
        isCrankPulling = false;
        isAlternating = true;
        setMotorSpeed(MOTOR_NORMAL);  // Attentive listening
        // NO MUSIC
        break;

      case 37:  // IDLE MODE (SW5) - NEW!
        Serial.println(F("IDLE: Calm Rainbow + Slow Motor"));
        if (crank.attached()) crank.detach();
        isCrankPulling = false;
        isFlashingRainbow = true;
        rainbowFlashStartTime = millis();
        lastRainbowUpdate = millis();
        rainbowOffset = 0;
        setMotorSpeed(MOTOR_IDLE);  // Gentle idle
        // NO MUSIC - just vibing
        break;

      default:
        Serial.println(F("Invalid case"));
        if (crank.attached()) crank.detach();
        setColor(0, 0, 0);
    }
  }
}

// end of loop()
// end of receiver code
// CHANGEHERE
*/

// Uncomment this to activate the receiver code
