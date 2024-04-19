#include <AccelStepper.h>

// Define constants for stepper motor pins
#define STEP_PIN 2
#define DIR_PIN 3
#define BUTTON_PIN 4

// Define variables for stepper motor control
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Define variables for button state
int buttonState = LOW;         // current state of the button
int lastButtonState = LOW;     // previous state of the button
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// Initialize initial seatment
uint16_t button_value = 1;

void setup() {
  // Set up stepper motor
  stepper.setMaxSpeed(100); // Set your desired maximum speed
  stepper.setAcceleration(500); // Set your desired acceleration
  
  // Set up button pin
  pinMode(BUTTON_PIN, OUTPUT); // Using internal pull-up resistor
  
  // Initialize serial communication
  Serial.begin(115200);

//  Serial.println(button_value, stepper.currentPosition());
//  Serial.print('[');
  Serial.print(button_value);
  Serial.print(',');
  Serial.println(stepper.currentPosition());
//  Serial.println(']');
}

void loop() {
  // Check button state with debounce
  int reading = digitalRead(BUTTON_PIN);
  
  // If the reading is different from the last state, reset the debounce timer
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // If the debounce time has passed, and the reading is still different, consider it valid
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has changed, update it
    if (reading != buttonState) {
      buttonState = reading;
      
      // If the button state is LOW, it's pressed
      if (buttonState == LOW) {
        // Cable unseated, send status to serial (value=0)
        button_value = 0;
//        Serial.println(button_value);
        delay(200);
        stepper.stop(); // Stop the motor
      } else {
        // Cable seated, send status to serial (value=1)
        button_value = 1;
//        Serial.println(button_value);
        stepper.stop(); // Stop the motor
      }
    }
  }
  
  lastButtonState = reading;

  // Publish button state and current stepper position to serial
//  Serial.println(button_value, stepper.currentPosition());
//  Serial.print('[');
  Serial.print(button_value);
  Serial.print(',');
  Serial.println(stepper.currentPosition());
//  Serial.println(']');

  // Check for serial commands
  if (Serial.available() >= 2) {
    uint16_t received_value;
    Serial.readBytes((char*)&received_value, 2);
    if (received_value == 2) {
      // Move motor in one direction (PULL)
      stepper.moveTo(1500);
    }
    if (received_value == 3) {
      // Move motor in the other direction (HOME)
      stepper.moveTo(-1500);
    }
    if (received_value == 4) {
      // Motor stop
      stepper.stop();
    }
  }

  // If motor is moving, update its position
  stepper.run();
}
