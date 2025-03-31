#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PWM servo driver object using custom address (0x41)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Define the servo parameters
#define SERVO_MIN_PWM  50 // This is the 'minimum' pulse length count (out of 4096)
#define SERVO_MAX_PWM  1000 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ     50  // Analog servos run at ~50 Hz

// Number of servo channels on PCA9685
#define NUM_SERVOS     16

// Direction and position tracking
int servoPos = 0;       // Current position in degrees
int servoDir = 1;       // Direction: 1 = increasing, -1 = decreasing
int servoStep = 1;      // Steps in degrees per loop
int servoDelay = 1;    // Delay between steps in milliseconds

void setup() {
  // Initialize I2C on the STM32F411
  // Use default I2C pins for Wire
  Wire.setSCL(PB_6);
  Wire.setSDA(PB_7);
  Wire.begin();
  
  // Initialize the PWM driver
  pwm.begin();
  
  // Set the PWM frequency for the servos
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(10);
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  // Convert position in degrees to pulse width
  uint16_t pulseWidth = map(servoPos, 0, 180, SERVO_MIN_PWM, SERVO_MAX_PWM);
  
  // Set PWM to move all servos simultaneously
  for (uint8_t servoNum = 0; servoNum < NUM_SERVOS; servoNum++) {
    pwm.setPWM(servoNum, 0, pulseWidth);
  }
  
  // Update position for next iteration
  servoPos += servoDir * servoStep;
  
  // Change direction when reaching limits
  if (servoPos >= 180) {
    servoPos = 180;
    servoDir = -1;
    digitalWrite(LED_BUILTIN, HIGH);
  } else if (servoPos <= 0) {
    servoPos = 0;
    servoDir = 1;
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  // Small delay to control the sweep speed
  delay(servoDelay);
}
