#include <AccelStepper.h>

// Define constants for stepper motor pins
#define STEP_PIN 2
#define DIR_PIN 3
#define BUTTON_PIN 4

// Define variables for stepper motor control
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Define variables for button state
int buttonState = 0;
int lastButtonState = 0;

// Initialize initial seatment
String command = "SEATED";

void setup() {
  // Set up stepper motor
  stepper.setMaxSpeed(200); // Set your desired maximum speed
  stepper.setAcceleration(500); // Set your desired acceleration
  
  // Set up button pin
  pinMode(BUTTON_PIN, OUTPUT);
  
  // Initialize serial communication
  Serial.begin(115200);

  Serial.println(command);
}

void loop() {
  // Check button state
  int reading = digitalRead(BUTTON_PIN);
  
  if (reading != lastButtonState) {
    if (reading == LOW) {
      // Button is pressed, send status to serial
      Serial.println("UNSEATED");
      stepper.stop(); // Stop the motor
    } else {
      // Button is released, send status to serial
      Serial.println("SEATED");
      stepper.stop(); // Stop the motor
    }
    delay(50); // Debounce delay
  }
  
  lastButtonState = reading;

  // Check for serial commands
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    if (command == "PULL") {
      // Move motor in one direction
      stepper.moveTo(1500);
//      stepper.run();
    }
    if (command == "HOME") {
      // Move motor in the other direction
      stepper.moveTo(-1500);
//      stepper.run();
    }
  }

  // If motor is moving, update its position
  stepper.run();
}
