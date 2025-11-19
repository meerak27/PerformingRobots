// Meera Al Khazraji & Vicor Nadu
// Professor Micheal Shiloh
// Performing Robots Fall '25
// Template of this code was taken from Professor Shiloh. We have attempted to mark all edits as we go along, but we will undoubtedly miss some.
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
// const int NRF_CE_PIN = A4, NRF_CSN_PIN = A5;

// CHANGEHERE
// for the receiver
const int NRF_CE_PIN = A11, NRF_CSN_PIN = A15;

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
// Rama and Hind Y: Channel 30, addr = 0x76
// Ahsen and Pranav: Channel 40, addr = 0x73
// Sara & Toomie:  Channel 50, addr = 0x7C
// Avinash and Vahagn: Channel 60, addr = 0xC6
// Hind A & Javeria:  Channel 70, addr = 0xC3
// Mbebo and Aaron: Channel 80, addr = 0xCC
// Linh and Luke: Channel 90, addr = 0x33

// CHANGEHERE
const byte CUSTOM_ADDRESS_BYTE = 0xCC;  // change as per the above assignment
const int CUSTOM_CHANNEL_NUMBER = 80;   // change as per the above assignment

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

// *** Victor Edits: Add a "busy" flag for the transmitter ***
bool isTransmitting = false;

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

/*

// Transmitter code


// Transmitter pin usage
const int LCD_RS_PIN = 3, LCD_EN_PIN = 2, LCD_D4_PIN = 4, LCD_D5_PIN = 5, LCD_D6_PIN = 6, LCD_D7_PIN = 7;
const int SW1_PIN = 8, SW2_PIN = 9, SW3_PIN = 10, SW4_PIN = A3, SW5_PIN = A2;

// LCD library code
#include <LiquidCrystal.h>

// initialize the library with the relevant pins
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

// MEERA EDIT
const int NUM_OF_STATES = 32;
char* theStates[] = {
  // Scene 1 (Tracks 1–9)
  "Scene 1.1",
  "Scene 1.2",
  "Scene 1.3",
  "Scene 1.4",
  "Scene 1.5",
  "Scene 1.6",
  "Scene 1.7",
  "Scene 1.8",
  "Scene 1.9",
  // Scene 3 (Tracks 10–17)
  "Scene 3.1",
  "Scene 3.2",
  "Scene 3.3",
  "Scene 3.4",
  "Scene 3.5",
  "Scene 3.6",
  "Scene 3.7",
  "Scene 3.8",
  // Scene 5 (Tracks 18–32)
  "Scene 5.1",
  "Scene 5.2",
  "Scene 5.3",
  "Scene 5.4",
  "Scene 5.5",
  "Scene 5.6",
  "Scene 5.7",
  "Scene 5.8",
  "Scene 5.9",
  "Scene 5.10",
  "Scene 5.11",
  "Scene 5.12",
  "Scene 5.13",
  "Scene 5.14",
  "Scene 5.15"
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
void spare1() {}
void spare2() {}

void rf24SendData() {

  // // *** Victor Edits: Prevent re-entry if already transmitting ***
  // if (isTransmitting) {
  //   Serial.println(F("Already transmitting, please wait."));
  //   // We can even tell the user on the LCD
  //   lcd.setCursor(0, 1);
  //   lcd.print("Busy...         ");
  //   return; // Exit the function immediately
  // }
  // isTransmitting = true; // Set the flag so we can't run again

  radio.stopListening();  // go into transmit mode
  // The write() function will block
  // until the message is successfully acknowledged by the receiver
  // or the timeout/retransmit maxima are reached.
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
  
  // *** Victor Edits: Clear the "busy" flag when done ***
  // isTransmitting = false;
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

*/

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


// *** Victor Edits: Add easy-to-edit variables for crank tuning ***
// *** Victor Edits: Renamed servo pin **
// This is your "up" or "resting" position. 0 is highest, 180 is lowest.
const int CRANK_REST_POSITION = 50;
// This is your "down" or "pulled" position.
const int CRANK_PULL_POSITION = 130;
// This controls the speed. Smaller number = faster.
const int crankMoveDelay = 15;   // ms between crank steps
const int CRANK_SERVO_PIN = 20;  // This is the pin for your crank servo

// Meera Edits
const int NEOPIXELPIN = 17;
const int NUMPIXELS = 31;
const int MOTOR_PIN = 8;  // DC motor control pin
bool isFlashingRed = false;
unsigned long redFlashStartTime = 0;
const unsigned long RED_FLASH_DURATION = 4500;
bool isFlashingRainbow = false;
bool isBreathing = false;
bool isPulsing = false;
bool isSparkle = false;
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
#define CARDCS 4         // Card chip select pin
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
// *** Victor Edits: Renamed servo object ***
Servo crank;  // This servo object will control your crank on pin 20


// *** Victor Edits: Add variables for non-blocking crank pull ***
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
  // *** Victor Edits: Use new crank name to attach ***
  crank.attach(CRANK_SERVO_PIN);     // Attaches the servo on pin 20
  crank.write(CRANK_REST_POSITION);  // Sets initial position
  //  antenna.attach(ANTENNA_SERVO_PIN);
  //  tail.attach(TAIL_SERVO_PIN);
  //  grabber.attach(GRABBER_SERVO_PIN);
  //
  //  tail.write(TAIL_HAPPY);
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
// *** Victor Edits: Fixed logic in updateCrankPull ***
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
  // Add effect updates
  updateBreathing(128, 0, 128);  // Purple breathing - adjust colors per case
  updatePulsing(255, 0, 100);    // Pink pulsing - adjust per case
  updateSparkle(255, 200, 0);    // Gold sparkle - adjust per case

  radio.startListening();
  if (radio.available(&pipeNum)) {
    radio.read(&data, sizeof(data));
    Serial.print(F("message received Data = "));
    Serial.print(data.stateNumber);
    Serial.println();

    // Reset all effects
    isBreathing = false;
    isPulsing = false;
    isSparkle = false;
    isFlashingRed = false;
    isFlashingRainbow = false;

    switch (data.stateNumber) {

        // ===== SCENE 1 =====

      case 0:  // "Well, that didn't take long..."
        Serial.println(F("Purple Breathing - Entrance"));
        // STOP any crank movement to prevent jitter
        isCrankPulling = false;
        isReturningToRest = false;
        if (crank.attached()) {
          crank.write(CRANK_REST_POSITION);
          crank.detach();
        }
        isBreathing = true;
        breathingBrightness = 0;
        breathingDirection = 1;
        musicPlayer.playFullFile("/track001.mp3");
        break;

      case 1:  // "That's my client. Faithful as a coin toss..."
        Serial.println(F("Gold - Money colors"));
        if (crank.attached()) crank.detach();
        setColor(255, 200, 0);  // Gold
        musicPlayer.playFullFile("/track002.mp3");
        break;

      case 2:  // "Over there, the wife..."
        Serial.println(F("Icy White - Clinical"));
        if (crank.attached()) crank.detach();
        setColor(255, 255, 255);  // White
        musicPlayer.playFullFile("/track003.mp3");
        break;

      case 3:  // "Poor kiddo, paternity is a jackpot..."
        Serial.println(F("Dark Red + Pull - Paternity bomb"));
        setColor(100, 0, 0);  // Dark red
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        if (CRANK_REST_POSITION < CRANK_PULL_POSITION) {
          crankPullDirection = 1;
        } else {
          crankPullDirection = -1;
        }
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        musicPlayer.playFullFile("/track004.mp3");
        break;

      case 4:  // "Flexible in every sense..."
        Serial.println(F("Hot Pink Pulsing - Sleaze"));
        if (crank.attached()) crank.detach();
        isPulsing = true;
        breathingBrightness = 255;
        breathingDirection = -1;
        musicPlayer.playFullFile("/track005.mp3");
        break;

      case 5:  // "Saulbot or something..."
        Serial.println(F("Green - Mockery"));
        if (crank.attached()) crank.detach();
        setColor(0, 255, 0);  // Green
        musicPlayer.playFullFile("/track006.mp3");
        break;

      case 6:  // "Our judge. Calm, composed..."
        Serial.println(F("Royal Blue - Respect"));
        if (crank.attached()) crank.detach();
        setColor(0, 0, 200);  // Royal blue
        musicPlayer.playFullFile("/track007.mp3");
        break;

      case 7:  // "This one's going to be fun."
        Serial.println(F("RAINBOW + BIG PULL - Game on!"));
        isFlashingRainbow = true;
        rainbowFlashStartTime = millis();
        lastRainbowUpdate = millis();
        rainbowOffset = 0;
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        if (CRANK_REST_POSITION < CRANK_PULL_POSITION) {
          crankPullDirection = 1;
        } else {
          crankPullDirection = -1;
        }
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        musicPlayer.playFullFile("/track008.mp3");
        break;

      case 8:  // "I can smell the deposit fees."
        Serial.println(F("Gold Sparkle - $$$"));
        if (crank.attached()) crank.detach();
        isSparkle = true;
        musicPlayer.playFullFile("/track009.mp3");
        break;

        // ===== SCENE 3 =====

      case 9:  // "Misunderstanding, huh? Real flexible."
        Serial.println(F("Orange - Word games"));
        if (crank.attached()) crank.detach();
        setColor(255, 100, 0);  // Orange
        musicPlayer.playFullFile("/track010.mp3");
        break;

      case 10:  // "Like your wife's best friend..."
        Serial.println(F("Hot Pink Pulse - Innuendo bomb"));
        if (crank.attached()) crank.detach();
        isPulsing = true;
        breathingBrightness = 255;
        breathingDirection = -1;
        musicPlayer.playFullFile("/track011.mp3");
        break;

      case 11:  // "Relax, I'm on your side..."
        Serial.println(F("Soft Blue Breathing - Fake comfort"));
        if (crank.attached()) crank.detach();
        isBreathing = true;
        breathingBrightness = 0;
        breathingDirection = 1;
        musicPlayer.playFullFile("/track012.mp3");
        break;

      case 12:  // "It's not a crime to dance..."
        Serial.println(F("Purple + White sparkles"));
        if (crank.attached()) crank.detach();
        for (int i = 0; i < strip.numPixels(); i++) {
          if (i % 3 == 0) {
            strip.setPixelColor(i, strip.Color(255, 255, 255));  // White (GRB)
          } else {
            strip.setPixelColor(i, strip.Color(0, 128, 128));  // Purple (GRB)
          }
        }
        strip.show();
        musicPlayer.playFullFile("/track013.mp3");
        break;

      case 13:  // "Let's make her pay..."
        Serial.println(F("BLOOD RED PULSE + PULL - Attack!"));
        isPulsing = true;
        breathingBrightness = 255;
        breathingDirection = -1;
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        if (CRANK_REST_POSITION < CRANK_PULL_POSITION) {
          crankPullDirection = 1;
        } else {
          crankPullDirection = -1;
        }
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        musicPlayer.playFullFile("/track014.mp3");
        break;

      case 14:  // "Play the victim..."
        Serial.println(F("Silver + PULL - Master manipulator"));
        setColor(180, 180, 180);  // Silver
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        if (CRANK_REST_POSITION < CRANK_PULL_POSITION) {
          crankPullDirection = 1;
        } else {
          crankPullDirection = -1;
        }
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        musicPlayer.playFullFile("/track015.mp3");
        break;

      case 15:  // "Kid's got a sense of humor."
        Serial.println(F("Cyan - Amused"));
        if (crank.attached()) crank.detach();
        setColor(0, 255, 255);  // Cyan
        musicPlayer.playFullFile("/track016.mp3");
        break;

      case 16:  // "Regret sells better than guilt."
        Serial.println(F("Purple+Gold + PULL - Philosophy"));
        for (int i = 0; i < strip.numPixels(); i++) {
          if (i % 2 == 0) {
            strip.setPixelColor(i, strip.Color(0, 128, 128));  // Purple (GRB)
          } else {
            strip.setPixelColor(i, strip.Color(200, 255, 0));  // Gold (GRB)
          }
        }
        strip.show();
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        if (CRANK_REST_POSITION < CRANK_PULL_POSITION) {
          crankPullDirection = 1;
        } else {
          crankPullDirection = -1;
        }
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        musicPlayer.playFullFile("/track017.mp3");
        break;

        // ===== SCENE 5 - COURTROOM =====

      case 17:  // Opening: "Casanova with a rap sheet"
        Serial.println(F("WHITE FLASH -> PURPLE + PULL - Opening!"));
        setColor(255, 255, 255);  // Flash white
        delay(200);
        setColor(0, 150, 150);  // Royal purple (GRB)
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        if (CRANK_REST_POSITION < CRANK_PULL_POSITION) {
          crankPullDirection = 1;
        } else {
          crankPullDirection = -1;
        }
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        musicPlayer.playFullFile("/track018.mp3");
        break;

      case 18:  // "Sure, he danced..."
        Serial.println(F("Orange - Dismissive"));
        if (crank.attached()) crank.detach();
        setColor(255, 100, 0);  // Orange
        musicPlayer.playFullFile("/track019.mp3");
        break;

      case 19:  // "Vegas casino at midnight"
        Serial.println(F("Gold + Red alternating - Vegas"));
        if (crank.attached()) crank.detach();
        for (int i = 0; i < strip.numPixels(); i++) {
          if (i % 2 == 0) {
            strip.setPixelColor(i, strip.Color(200, 255, 0));  // Gold (GRB)
          } else {
            strip.setPixelColor(i, strip.Color(0, 255, 0));  // Red (GRB)
          }
        }
        strip.show();
        musicPlayer.playFullFile("/track020.mp3");
        break;

      case 20:  // "Opposing counsel Saulbot..."
        Serial.println(F("Green Pulsing - Mocking rival"));
        if (crank.attached()) crank.detach();
        isPulsing = true;
        breathingBrightness = 255;
        breathingDirection = -1;
        musicPlayer.playFullFile("/track021.mp3");
        break;

      case 21:  // "Preacher on Sunday, bachelor on Friday"
        Serial.println(F("White -> Pink Flash - Holy to sleazy"));
        setColor(255, 255, 255);  // White
        delay(300);
        setColor(100, 255, 100);  // Pink (GRB)
        if (crank.attached()) crank.detach();
        musicPlayer.playFullFile("/track022.mp3");
        break;

      case 22:  // "Harassment claims alphabetically"
        Serial.println(F("BRIGHT RED + PULL - Scandal bomb!"));
        setColor(0, 255, 0);  // Bright red (GRB)
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        if (CRANK_REST_POSITION < CRANK_PULL_POSITION) {
          crankPullDirection = 1;
        } else {
          crankPullDirection = -1;
        }
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        musicPlayer.playFullFile("/track023.mp3");
        break;

      case 23:  // "Your Honor, I trust your judgment..."
        Serial.println(F("Royal Blue Breathing - Flattery"));
        if (crank.attached()) crank.detach();
        isBreathing = true;
        breathingBrightness = 0;
        breathingDirection = 1;
        musicPlayer.playFullFile("/track024.mp3");
        break;

      case 24:  // "A bit of dancing, a lot of exaggeration..."
        Serial.println(F("Soft Purple - Reasonable"));
        if (crank.attached()) crank.detach();
        setColor(0, 100, 100);  // Soft purple (GRB)
        musicPlayer.playFullFile("/track025.mp3");
        break;

      case 25:  // "Now, tell us, kid..."
        Serial.println(F("Yellow - Interrogation"));
        if (crank.attached()) crank.detach();
        setColor(255, 255, 0);  // Yellow
        musicPlayer.playFullFile("/track026.mp3");
        break;

      case 26:  // "Did your father look like..."
        Serial.println(F("Gentle Blue - Leading question"));
        if (crank.attached()) crank.detach();
        setColor(150, 100, 255);  // Gentle blue (GRB)
        musicPlayer.playFullFile("/track027.mp3");
        break;

      case 27:  // "What would you say your father was thinking?"
        Serial.println(F("Teal - Probing"));
        if (crank.attached()) crank.detach();
        setColor(0, 200, 150);  // Teal (GRB - swapped)
        musicPlayer.playFullFile("/track028.mp3");
        break;

      case 28:  // "Kids say the darndest things..."
        Serial.println(F("Nervous Orange Flicker"));
        if (crank.attached()) crank.detach();
        isSparkle = true;
        musicPlayer.playFullFile("/track029.mp3");
        break;

      case 29:  // "If you had to choose..."
        Serial.println(F("Dark Red - The trap"));
        if (crank.attached()) crank.detach();
        setColor(0, 150, 0);  // Dark red (GRB)
        musicPlayer.playFullFile("/track030.mp3");
        break;

      case 30:  // "But I meant between your mother and father."
        Serial.println(F("Yellow/White flicker - Panic"));
        if (crank.attached()) crank.detach();
        for (int i = 0; i < strip.numPixels(); i++) {
          if (random(2) == 0) {
            strip.setPixelColor(i, strip.Color(255, 255, 255));  // White
          } else {
            strip.setPixelColor(i, strip.Color(255, 255, 0));  // Yellow
          }
        }
        strip.show();
        musicPlayer.playFullFile("/track031.mp3");
        break;

      case 31:  // "JACKPOT!"
        Serial.println(F("EXPLOSIVE RAINBOW + MAX PULL - VICTORY!!!"));
        // Flash sequence
        setColor(255, 255, 255);
        delay(100);
        setColor(0, 0, 0);
        delay(50);
        setColor(255, 255, 255);
        delay(100);
        // Then rainbow
        isFlashingRainbow = true;
        rainbowFlashStartTime = millis();
        lastRainbowUpdate = millis();
        rainbowOffset = 0;
        if (!crank.attached()) crank.attach(CRANK_SERVO_PIN);
        isCrankPulling = true;
        isReturningToRest = false;
        if (CRANK_REST_POSITION < CRANK_PULL_POSITION) {
          crankPullDirection = 1;
        } else {
          crankPullDirection = -1;
        }
        currentCrankAngle = CRANK_REST_POSITION;
        lastCrankMoveTime = millis();
        musicPlayer.playFullFile("/track032.mp3");
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


// Uncomment this to activate the receiver code
