/*
Federal University of Bahia (UFBA)
Department of Electrical and Computer Engineering (DEEC) - Polytechnic School
Students: Gabriel Correia and Mateus Fiscina


CONVEYOR SPEED CONTROL MODEL USING PID
*/

// ---------------- DEFINITION AND INITIAL SETTINGS ----------------
// Import Code Libraries (important to download before running)
#include <AFMotor.h> // Library for controlling DC motors using the Adafruit Motor Shield
#include <PID_v1.h>  // Library for PID control
#include <Wire.h>    // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for I2C LCD control

// Engine configuration
AF_DCMotor motor(1); // Creates a motor object on port M1 of the Adafruit Motor Shield

// PID Control Constants
#define MIN_PWM 0   // Minimum PWM value
#define MAX_PWM 255 // Maximum PWM value
#define KP 0.1625  // PID proportional constant
#define KI 1.776   // PID integral constant
#define KD 0       // PID derivative constant

// Variables for the Infrared Sensor and PID
double rpm;                  // Stores the current motor RPM value
volatile byte pulses;        // Pulse counter for the infrared sensor
unsigned long timeold;       // Marks the time of the last RPM update
int sensorPin = 18;          // Arduino pin connected to the D0 pin of the sensor
unsigned int discPulses = 20; // Number of pulses per full rotation of the encoder disc
double speed = 0;            // Stores the PID output value (PWM)
double speedSetpoint = 100;  // Desired speed (setpoint) in RPM

// Constant for the potentiometer pin
const int potPin = A10;   // Analog pin where the potentiometer is connected
const int switchPin = 30; // Digital pin where the switch is connected

// Replace 0x27 with the address of your LCD found by the I2C scanner
LiquidCrystal_I2C lcd(0x27, 16, 2); // Initializes the I2C LCD display with address 0x27, 16 columns, and 2 rows

// Creates PID for control
PID motorPID(&rpm, &speed, &speedSetpoint, KP, KI, KD, DIRECT);


// ---------------- PROGRAM TO BE RUN ----------------
// Function executed at each interrupt
void counter() {
  pulses++;  // Increments the pulse counter
}

// SETUP
void setup() {
  // Start Serial
  Serial.begin(9600); // Starts serial communication at 9600 bps

  // Start the I2C LCD
  lcd.init(); // Initializes the LCD display
  // lcd.begin(16, 2);
  lcd.backlight(); // Turns on the LCD backlight
 
  // Interrupt Configuration
  pinMode(sensorPin, INPUT);  // Sets the sensor pin as input
  pinMode(switchPin, INPUT);  // Configures the switch pin as input with an internal pull-up

  attachInterrupt(digitalPinToInterrupt(sensorPin), counter, FALLING); // Sets an interrupt to count pulses on the falling edge
  pulses = 0;    // Initializes the pulse counter
  rpm = 0;       // Initializes the RPM value
  timeold = 0;   // Initializes the previous time

  // PID Control Configuration
  motorPID.SetOutputLimits(MIN_PWM, MAX_PWM); // Sets the PID output limits
  motorPID.SetMode(AUTOMATIC); // Sets the PID mode to automatic
}


// LOOP
void loop() {
  // Reads the potentiometer value and adjusts the speed setpoint
  int potValue = analogRead(potPin); // Reads the potentiometer value
  speedSetpoint = map(potValue, 0, 1023, 0, 500); // Maps the potentiometer value to the desired RPM range

  // Reads the switch state
  bool switchState = digitalRead(switchPin); // Reads the switch state (HIGH if pressed, LOW if not pressed)

  // Calculates RPM every 1 Second
  if (millis() - timeold >= 1000) { // If one second has passed since the last measurement
    detachInterrupt(digitalPinToInterrupt(sensorPin)); // Disables the interrupt during the calculation to avoid inconsistencies
    rpm = (60 * 1000 / discPulses) / (millis() - timeold) * pulses; // Calculates RPM based on the counted pulses
    timeold = millis(); // Updates the previous time to the current time
    pulses = 0; // Resets the pulse counter
    
    // Displays ALL values on the serial monitor
    Serial.print("Setpoint: ");
    Serial.print(speedSetpoint); // Displays the current speed setpoint
    Serial.print("  ");
    Serial.print("Speed - PID output: ");
    Serial.print(speed, 2); // Displays the speed calculated by the PID with two decimal places
    Serial.print("  ");
    Serial.print("RPM: ");
    Serial.println(rpm, 0); // Displays the calculated RPM value
    Serial.print("Error: ");
    Serial.println(speedSetpoint - rpm); // Displays the error between the setpoint and the RPM measured by the sensor

    // Re-enables the interrupt
    attachInterrupt(digitalPinToInterrupt(sensorPin), counter, FALLING);
  }

  // Calculates the motor PWM according to the PID control
  motorPID.Compute(); // Calculates the PID output value based on the current RPM and setpoint

  // Reads the switch state
  //int switchState = digitalRead(switchPin);

  // Adjusts motor PWM based on the switch state
  if (switchState == LOW) {
    motor.run(FORWARD); // If the switch is pressed, runs the motor forward
   
  } else {
    motor.run(BACKWARD); // Otherwise, runs the motor backward
    
  }
  
  motor.setSpeed(speed);   // Sets the motor PWM with the value calculated by the PID

  // Displays the parameters on the I2C LCD screen
  lcd.clear(); // Clears the LCD display
  lcd.setCursor(0, 0); // Moves the cursor to the first line
  lcd.print("Setpoint: "); // Displays "Setpoint:" on the LCD
  lcd.print(speedSetpoint); // Displays the current speed setpoint
  lcd.setCursor(0, 1); // Moves the cursor to the second line
  lcd.print("Sensor: "); // Displays "Sensor:" on the LCD
  lcd.print(rpm);      // Displays the calculated RPM value
  lcd.print(" RPM");   // Displays " RPM" after the RPM value
  //delay(100); // Waits 100 milliseconds before repeating the loop
}
