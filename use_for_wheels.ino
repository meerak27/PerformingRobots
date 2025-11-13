// Meera AlKhazraji
// 13 Nov
// must check if backwards is now more stable 

#include <EnableInterrupt.h>
#define SERIAL_PORT_SPEED 9600
#define RC_NUM_CHANNELS 4
#define RC_CH1 0
#define RC_CH2 1
#define RC_CH3 2
#define RC_CH4 3
// pin usage // radio pins
#define RC_CH1_PIN 6
#define RC_CH2_PIN 5
#define RC_CH3_PIN 4
#define RC_CH4_PIN 3
// motor driver pins
const int EN1_PIN = 9;
const int IN1_PIN = 8;
const int IN2_PIN = 7;
const int IN3_PIN = 10;
const int IN4_PIN = 12;
const int EN2_PIN = 11;

int currentPWM_L = 0;               // Left motor: -255..255
int currentPWM_R = 0;               // Right motor: -255..255
const int PWM_MAX = 255;
const float REVERSE_SPEED_LIMIT = 0.7;  // Limit reverse to 70% of forward speed (0.0 to 1.0)
const int ACCEL_STEP = 20;          // Acceleration step
const int DECEL_STEP = 35;          // Deceleration step (higher = smoother stops)
const unsigned long DT = 15;
unsigned long lastUpdate = 0;


uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() {
  calc_input(RC_CH1, RC_CH1_PIN);
}
void calc_ch2() {
  calc_input(RC_CH2, RC_CH2_PIN);
}
void calc_ch3() {
  calc_input(RC_CH3, RC_CH3_PIN);
}
void calc_ch4() {
  calc_input(RC_CH4, RC_CH4_PIN);
}

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  pinMode(RC_CH1_PIN, INPUT); //side wheel
  pinMode(RC_CH2_PIN, INPUT); //trigger
  pinMode(RC_CH3_PIN, INPUT); //side knob
  pinMode(RC_CH4_PIN, INPUT); //bottom button
  enableInterrupt(RC_CH1_PIN, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_PIN, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_PIN, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_PIN, calc_ch4, CHANGE);
  pinMode(9, OUTPUT); //en1
  pinMode(8, OUTPUT); //in1
  pinMode(7, OUTPUT); //in2
  pinMode(11, OUTPUT); //en2
  pinMode(10, OUTPUT); //in3
  pinMode(12, OUTPUT); //in4
}

void loop() {
  rc_read_values();

  // --- Read throttle from CH1 (side wheel) ---
  int throttle = 0;
  if (rc_values[RC_CH1] > 1600) {
    throttle = ((long)(rc_values[RC_CH1] - 1600) * PWM_MAX) / 700;
    throttle = constrain(throttle, 0, PWM_MAX);
  } else if (rc_values[RC_CH1] < 1400) {
    throttle = -((long)(1400 - rc_values[RC_CH1]) * PWM_MAX) / 700;
    throttle = constrain(throttle, -PWM_MAX, 0);
    // Apply reverse speed limit
    throttle = (int)(throttle * REVERSE_SPEED_LIMIT);
  } else {
    throttle = 0;
  }

  // --- Read steering from CH2 (trigger) ---
  int steering = 0;
  if (rc_values[RC_CH2] > 1600) {
    steering = ((long)(rc_values[RC_CH2] - 1600) * PWM_MAX) / 700;
    steering = constrain(steering, 0, PWM_MAX);
  } else if (rc_values[RC_CH2] < 1400) {
    steering = -((long)(1400 - rc_values[RC_CH2]) * PWM_MAX) / 700;
    steering = constrain(steering, -PWM_MAX, 0);
  } else {
    steering = 0;
  }

  // --- Calculate desired speeds for left and right motors ---
  int desired_L = throttle - steering;
  int desired_R = throttle + steering;
  
  desired_L = constrain(desired_L, -PWM_MAX, PWM_MAX);
  desired_R = constrain(desired_R, -PWM_MAX, PWM_MAX);

  // --- Improved slew limiter for both motors ---
  unsigned long now = millis();
  if (now - lastUpdate >= DT) {
    // Left motor
    int diff_L = desired_L - currentPWM_L;
    
    // Check if we're accelerating or decelerating
    // Decelerating means moving toward zero OR changing direction
    bool isDecel_L = (abs(desired_L) < abs(currentPWM_L)) || 
                     ((desired_L >= 0) != (currentPWM_L >= 0) && currentPWM_L != 0);
    
    int step_L = isDecel_L ? DECEL_STEP : ACCEL_STEP;
    
    if (abs(diff_L) <= step_L) {
      currentPWM_L = desired_L;  // Close enough, snap to target
    } else if (diff_L > 0) {
      currentPWM_L += step_L;
    } else {
      currentPWM_L -= step_L;
    }

    // Right motor
    int diff_R = desired_R - currentPWM_R;
    
    bool isDecel_R = (abs(desired_R) < abs(currentPWM_R)) || 
                     ((desired_R >= 0) != (currentPWM_R >= 0) && currentPWM_R != 0);
    
    int step_R = isDecel_R ? DECEL_STEP : ACCEL_STEP;
    
    if (abs(diff_R) <= step_R) {
      currentPWM_R = desired_R;  // Close enough, snap to target
    } else if (diff_R > 0) {
      currentPWM_R += step_R;
    } else {
      currentPWM_R -= step_R;
    }

    // --- Apply to left motor (Motor 1) ---
    int pwm_L = abs(currentPWM_L);
    if (currentPWM_L >= 0) {
      digitalWrite(IN1_PIN, LOW);
      digitalWrite(IN2_PIN, HIGH);
    } else {
      digitalWrite(IN1_PIN, HIGH);
      digitalWrite(IN2_PIN, LOW);
    }
    analogWrite(EN1_PIN, pwm_L);

    // --- Apply to right motor (Motor 2) ---
    int pwm_R = abs(currentPWM_R);
    if (currentPWM_R >= 0) {
      digitalWrite(IN3_PIN, LOW);
      digitalWrite(IN4_PIN, HIGH);
    } else {
      digitalWrite(IN3_PIN, HIGH);
      digitalWrite(IN4_PIN, LOW);
    }
    analogWrite(EN2_PIN, pwm_R);

    lastUpdate = now;
  }

  delay(5);
}

void forward(int value) {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  analogWrite(EN1_PIN, value);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  analogWrite(EN2_PIN, value);
}

void reverse(int value) {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(EN1_PIN, value);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(EN2_PIN, value);
}

void right(int value) {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  analogWrite(EN1_PIN, value);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(EN2_PIN, value);
}

void left(int value) {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(EN1_PIN, value);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  analogWrite(EN2_PIN, value);
}

void stop() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(EN1_PIN, 0);
  analogWrite(EN2_PIN, 0);
  currentPWM_L = 0;
  currentPWM_R = 0;
}
