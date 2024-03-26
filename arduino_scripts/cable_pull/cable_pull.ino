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

void setup() {
  // Set up stepper motor
  stepper.setMaxSpeed(1000); // Set your desired maximum speed
  stepper.setAcceleration(500); // Set your desired acceleration
  
  // Set up button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize serial communication
  Serial.begin(115200);
}

void loop() {
  // Check button state
  int reading = digitalRead(BUTTON_PIN);
  
  if (reading != lastButtonState) {
    if (reading == LOW) {
      // Button is pressed, send status to serial
      Serial.println("SEATED");
    } else {
      // Button is released, send status to serial
      Serial.println("UNSEATED");
      stepper.stop(); // Stop the motor
    }
    delay(50); // Debounce delay
  }
  
  lastButtonState = reading;

  // Check for serial commands
  while (Serial.available() > 0) {
    String command = Serial.readString();
    if (command == "PULL") {
      // Move motor in one direction
      stepper.moveTo(1000); // Adjust the steps according to your requirement
    } else if (command == "HOME") {
      // Move motor in the other direction
      stepper.moveTo(-1000); // Adjust the steps according to your requirement
    }
  }

  // If motor is moving, update its position
  stepper.run();
}
