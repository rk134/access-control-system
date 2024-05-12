// Author: Rahul Kantrapally (20248129)
// Title: Access control system for clean room.
// Creation date: 17.04.2024
// Modification date: 10.05.2024
// Version: v5

/* Changes:
 * v1.0 - Initial program (17.04.2024)
 * v2.0 - Drop RGB LEDs and switch to conventional LEDs. (18.04.2024)
 * v3.0 - Re-work condition checking and ensure that all conditions are checked. (18.04.2024)
 * v3.1 - Clean-up + General commenting. (18.04.2024)
 * v3.2 - Ran code through beautifer/formatter. (18.04.2024)
 * v3.3 - Drop excessive exit button status checking (19.04.2024)
 * v4.0 - Use constant int where possible to avoid accidentally writing different value. (19.04.2024)
 * v4.1 - Adjust delays. (19.04.2024)
 * v5 - Overhaul of program.
 */

// Note: Servo angles determined from manual testing. Extracted from v5 (old).
// Required libraries: IRremote & Servo library

/* Includes */
#include <Servo.h> // Servo library

#include <IRremote.h> // IR library v3

/*------------------------------------- Defines -------------------------------------------*/
// Set to true if debugging is enabled.
bool DEBUG = false;

// Set port IDs for LED pins. (variables with an 'E' fixation indicates that it is external.)
const int greenPin = 0;
const int greenPinE = 1;
const int redPin = 2;
const int redPinE = 3;

// Define port IDs for Buzzer.
const int buzzerPin = 5;
const int soundDuration = 20000;

// Define port ID for 'exit' button.
bool exitButton;
const int buttonPin = 7;

// Define port ID for PIR Pin.
const int pirPinA = 8;
const int pirPinB = 10;
int sensorValueB; // Empty variables. Written to later in program.
int sensorValueA;

// Define port ID for PWM signal out for servo.
const int doorServoPin = 19;
const int lockServoPinB = 21;
Servo myservoDoor; // Setting alias
Servo myservoLock;

/*------------------------------------- Setup -------------------------------------------*/
// Setup program. (Runs only once)
void setup() {
  // Setting baud rate for serial console. 9600 is standard for UART.
  Serial.begin(9600);
  Serial.println("Initializing program... | Version v5 | Date of modification: 10.05.2024");

  // Enable PULLUP_RESISTORs for requiredPin components.
  pinMode(pirPinA, INPUT_PULLUP);
  pinMode(pirPinB, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);

  // We want the status LED to always be on. (Pin 12 - ref Blink documentation)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Test LED function
  pinMode(greenPin, OUTPUT);
  pinMode(greenPinE, OUTPUT);

  pinMode(redPin, OUTPUT);
  pinMode(redPinE, OUTPUT);

  digitalWrite(greenPin, HIGH);
  digitalWrite(greenPinE, HIGH);

  digitalWrite(redPin, HIGH);
  digitalWrite(redPinE, HIGH);

  delay(1000);

  digitalWrite(greenPin, LOW);
  digitalWrite(greenPinE, LOW);

  digitalWrite(redPin, LOW);
  digitalWrite(redPinE, LOW);

  // Test buzzer function
  pinMode(buzzerPin, OUTPUT);
  tone(buzzerPin, 1000); // 1 Khz signal
  delay(1000);
  noTone(buzzerPin); // Stop sound

  // Test doorServo function
  myservoDoor.attach(19); // Attach to pin
  myservoDoor.write(159); // Open
  delay(100);
  myservoDoor.write(43); // Closed

  // Test lockServo function
  myservoLock.attach(20); // Attach to pin
  myservoLock.write(100); // Unlock
  delay(100);
  myservoLock.write(180); // Lock

  // DEBUG: Print servo angles.
  if (DEBUG == true) {
    Serial.println(myservoDoor.read());
    Serial.println(myservoLock.read());
  }

  Serial.println("Waiting for PIR sensors");
  delay(30000);
}

// Loop main program.
void loop() {

  // Continously read sensor values.
  sensorValueA = digitalRead(pirPinA);
  sensorValueB = digitalRead(pirPinB);

  // On boot-up, no one is going to be inside the room.
  digitalWrite(greenPin, HIGH);
  digitalWrite(greenPinE, HIGH);

  digitalWrite(redPin, LOW);
  digitalWrite(redPinE, LOW);

  // Check for door position
  if (myservoDoor.read() != 43) {
    Serial.println("Door is not closed properly!");
    delay(20000); // 20s wait
    for (int i = 0; i < 1000; i++) {
      digitalWrite(greenPin, LOW);
      digitalWrite(greenPinE, LOW);
      digitalWrite(redPin, HIGH);
      digitalWrite(redPinE, HIGH);
      digitalWrite(redPin, LOW);
      digitalWrite(redPinE, LOW);
      digitalWrite(greenPin, HIGH);
      digitalWrite(greenPinE, HIGH);
    }
    tone(buzzerPin, 1000, soundDuration); // 1 Khz signal
    noTone(buzzerPin); // Stop sound
  }

  // DEBUG: Print values of sensors.
  if (DEBUG == true) {
    Serial.println(sensorValueA);
    Serial.println(sensorValueB);
  }

  // Condition based checking starts here.
  if (sensorValueA == HIGH && sensorValueB == LOW) { // External sensor tripped

    Serial.println("PIR (A) Triggered! Presence in front of door... Opening door.");

    // Set LEDs to RED, to indicate that there is someone going inside the room.
    digitalWrite(greenPin, LOW);
    digitalWrite(greenPinE, LOW);

    digitalWrite(redPin, HIGH);
    digitalWrite(redPinE, HIGH);

    // Open and close the door
    myservoLock.write(100); // Lock unlocked.
    delay(1000);
    myservoDoor.write(159);
    delay(1000);
    myservoDoor.write(43);
    myservoLock.write(180); // locked.
    Serial.println("Door closed, room occupied");

    // Exit button check
    while (exitButton != true) {
      if (digitalRead(buttonPin) == LOW) {
        Serial.println("Exit button pressed. Opening door.");

        // Indicators go Green.
        digitalWrite(greenPin, HIGH);
        digitalWrite(greenPinE, HIGH);
        digitalWrite(redPin, LOW);
        digitalWrite(redPinE, LOW);

        myservoLock.write(100); // Lock unlocked.
        delay(1000);
        myservoDoor.write(159); // Open
        delay(1000);
        myservoDoor.write(43); // Closed

        Serial.println("Door closed.");

        exitButton == true;

        break;
      } else if (digitalRead(buttonPin) != HIGH) {
        Serial.println("Door closed, waiting for exit button.");
        exitButton == false;
      }
    }

  } else if (sensorValueA == LOW && sensorValueB == LOW) { // Both sensors tripped

    Serial.println("PIR (A) triggered, but PIR (B) also triggered. Presence inside room. Do not open doors.");

    // LEDs go RED.
    digitalWrite(greenPin, LOW);
    digitalWrite(greenPinE, LOW);

    digitalWrite(redPin, HIGH);
    digitalWrite(redPinE, HIGH);

    // Keep door shut
    myservoDoor.write(43);
    delay(1000);
    myservoLock.write(180); // locked.

    // Flash internal indicator and play buzzer to indicate someone is outside.
    Serial.println("Sensor triggered, but room occupied");
    digitalWrite(redPin, LOW);
    digitalWrite(redPin, HIGH);
    digitalWrite(buzzerPin, HIGH);
    delay(1000);
    digitalWrite(buzzerPin, LOW);

    // Exit button check
    while (exitButton != true) {
      if (digitalRead(buttonPin) == LOW) {
        Serial.println("Exit button pressed. Opening door.");

        // Indicators go Green.
        digitalWrite(greenPin, HIGH);
        digitalWrite(greenPinE, HIGH);
        digitalWrite(redPin, LOW);
        digitalWrite(redPinE, LOW);

        myservoLock.write(100); // Lock unlocked.
        delay(1000);
        myservoDoor.write(159); // Open
        delay(1000);
        myservoDoor.write(43); // Closed

        Serial.println("Door closed.");

        exitButton == true;

        break;
      } else if (digitalRead(buttonPin) != HIGH) {
        Serial.println("Door closed, waiting for exit button.");
        exitButton == false;
      }
    }

  } else if (sensorValueA == HIGH && sensorValueB == LOW) { // Internal sensor tripped.

    Serial.println("PIR (A) not triggered, but PIR (B) triggered. Presence inside room. Do not open doors.");

    // LEDs go RED.
    digitalWrite(greenPin, LOW);
    digitalWrite(greenPinE, LOW);

    digitalWrite(redPin, HIGH);
    digitalWrite(redPinE, HIGH);

    // Keep door shut
    myservoDoor.write(43);
    delay(1000);
    myservoLock.write(180); // locked.

    // Exit button check
    while (exitButton != true) {
      if (digitalRead(buttonPin) == LOW) {
        Serial.println("Exit button pressed. Opening door.");

        // Indicators go Green.
        digitalWrite(greenPin, HIGH);
        digitalWrite(greenPinE, HIGH);
        digitalWrite(redPin, LOW);
        digitalWrite(redPinE, LOW);

        myservoLock.write(100); // Lock unlocked.
        delay(1000);
        myservoDoor.write(159); // Open
        delay(1000);
        myservoDoor.write(43); // Closed

        Serial.println("Door closed.");

        exitButton == true;

        break;
      } else if (digitalRead(buttonPin) != HIGH) {
        Serial.println("Door closed, waiting for exit button.");
        exitButton == false;
      }
    }

  } else if (sensorValueA == HIGH || sensorValueB == HIGH) { // Both sensors not tripped. Room is free!

    Serial.println("PIR (A) and PIR (B) not triggered. Room is free.");

    // Okay, we can go green now.
    digitalWrite(greenPin, HIGH);
    digitalWrite(greenPinE, HIGH);

    digitalWrite(redPin, LOW);
    digitalWrite(redPinE, LOW);

    myservoDoor.write(43); // Door is closed.
    myservoLock.write(100); // Door is unlocked

    Serial.println("Door closed and unlocked.");
    break; // We want to break the loop here, so that we can go back to the top.
  }

}