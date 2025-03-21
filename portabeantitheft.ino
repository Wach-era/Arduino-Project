#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SoftwareSerial.h>

// Pin definitions
#define BUZZER_PIN 6       // Buzzer connected to digital pin 6
#define BUTTON_PIN 7       // Button connected to digital pin 7
#define GREEN_LED_PIN 3  // Green LED for Accelerometer-Only Mode
#define RED_LED_PIN 4    // Red LED for Bluetooth-Only Mode
#define YELLOW_LED_PIN 5 // Yellow LED for Combined Mode

// Accelerometer object
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Bluetooth setup
SoftwareSerial BTSerial(10, 11); // RX (pin 10) | TX (pin 11)

// Mode enumeration
enum Mode { ACCELEROMETER_ONLY, BLUETOOTH_ONLY, COMBINED, DISABLED };
Mode currentMode = ACCELEROMETER_ONLY;

// Variables for Bluetooth connection
unsigned long lastHeartbeatTime = 0;
const unsigned long heartbeatTimeout = 10000; // 10-second timeout
bool isConnected = false;

// Button debouncing variables
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // Debounce delay in milliseconds
int buttonState = HIGH;
int lastButtonState = HIGH;

void setup() {
    Serial.begin(9600);
    
    // Initialize buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // Start with buzzer off

    // Initialize LEDs
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, LOW);


    // Initialize accelerometer
    if (!accel.begin()) {
        Serial.println("ADXL345 not detected. Check wiring!");
        while (1);
    }
    accel.setRange(ADXL345_RANGE_16_G); // Set sensitivity

    // Initialize Bluetooth
    BTSerial.begin(9600);

    // Initialize button
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Play initial mode feedback
    playModeFeedback();
    updateLEDs(); // Update LEDs based on the initial mode
}

void loop() {
    // Check button press to cycle modes
    int reading = digitalRead(BUTTON_PIN);

    // Debounce the button
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != buttonState) {
            buttonState = reading;

            // If the button is pressed (LOW), cycle modes
            if (buttonState == LOW) {
                currentMode = (Mode)((currentMode + 1) % 4); // Cycle through modes
                playModeFeedback(); // Play feedback beep for the new mode
                updateLEDs();
            }
        }
    }

    lastButtonState = reading;

    // Execute current mode
    switch (currentMode) {
        case ACCELEROMETER_ONLY:
            monitorAccelerometer();
            break;
        case BLUETOOTH_ONLY:
            monitorBluetooth();
            break;
        case COMBINED:
            monitorAccelerometer();
            monitorBluetooth();
            break;
        case DISABLED:
            // Do nothing (both alarms disabled)
            break;
    }

    delay(500); // Small delay to reduce CPU usage
}

void monitorAccelerometer() {
    sensors_event_t event;
    accel.getEvent(&event);

    // Detect sudden movement 
    if (abs(event.acceleration.x) > 10 || abs(event.acceleration.y) > 10 || abs(event.acceleration.z) > 10) {
        Serial.println("🚨 Motion Detected! Activating Alarm...");
        tone(BUZZER_PIN, 3000); 
        delay(50000); 
        noTone(BUZZER_PIN); // Stop sound
    }
}

void monitorBluetooth() {
    // Check if data is available from the HC-05
    if (BTSerial.available()) {
        char data = BTSerial.read(); // Read data from HC-05

        // Check if the data is a heartbeat signal
        if (data == 'H') {
            if (!isConnected) {
                isConnected = true; // Update connection status
                Serial.println("Bluetooth Connected!");
                noTone(BUZZER_PIN); // Stop the buzzer when connection is restored
            }
            lastHeartbeatTime = millis(); // Update last heartbeat time
        }
    }

    // Check if the connection is lost
    if (isConnected && millis() - lastHeartbeatTime > heartbeatTimeout) {
        isConnected = false; // Update connection status
        Serial.println("Bluetooth Disconnected! Activating Alarm...");
        tone(BUZZER_PIN, 3000); // Continuously sound the buzzer
    }
}

void playModeFeedback() {
    switch (currentMode) {
        case ACCELEROMETER_ONLY:
            tone(BUZZER_PIN, 1000, 200); // One short beep
            Serial.println("Accelerometer Mode");
            break;
        case BLUETOOTH_ONLY:
            tone(BUZZER_PIN, 1000, 200); // Two short beeps
            delay(100);
            tone(BUZZER_PIN, 1000, 200);
            Serial.println("Bluetooth Mode");
            break;
        case COMBINED:
            tone(BUZZER_PIN, 1000, 200); // Three short beeps
            delay(100);
            tone(BUZZER_PIN, 1000, 200);
            delay(100);
            tone(BUZZER_PIN, 1000, 200);
            Serial.println("Combined Mode");
            break;
        case DISABLED:
            tone(BUZZER_PIN, 1000, 1000); // One long beep
            Serial.println("Disabled");
            break;

    }
}

void updateLEDs() {
    // Turn off all LEDs
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, LOW);

    // Turn on the appropriate LED based on the current mode
    switch (currentMode) {
        case ACCELEROMETER_ONLY:
            digitalWrite(GREEN_LED_PIN, HIGH); // Green LED for Accelerometer-Only Mode
            break;
        case BLUETOOTH_ONLY:
            digitalWrite(RED_LED_PIN, HIGH); // Red LED for Bluetooth-Only Mode
            break;
        case COMBINED:
            digitalWrite(YELLOW_LED_PIN, HIGH); // Yellow LED for Combined Mode
            break;
        case DISABLED:
            // No LED for Disabled Mode
            break;
    }
}