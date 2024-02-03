const int MOTION_SENSOR_PIN = 2;
const int PIR_BUTTON_PIN = 7;       // the number of the pushbutton pin for PIR
const int SOIL_MOISTURE_BUTTON_PIN = 8;  // the number of the pushbutton pin for soil moisture
const int LED_PIN = 3;             // the number of the LED pin
const int ANALOG_SENSOR_PIN = 0;   // the analog sensor pin

volatile int motionSensorState = 0;  // variable for reading the motion sensor status (volatile for interrupts)
volatile int soilMoistureState = 0;  // variable for reading the soil moisture status (volatile for interrupts)
volatile bool sensorsEnabled = true; // flag to indicate whether sensors are enabled

void setup() {
  // initialize the LED pin as an output:
  pinMode(LED_PIN, OUTPUT);
  
  // initialize the pushbutton pin for PIR as an input with pull-up resistor:
  pinMode(PIR_BUTTON_PIN, INPUT_PULLUP);

  // initialize the pushbutton pin for soil moisture as an input with pull-up resistor:
  pinMode(SOIL_MOISTURE_BUTTON_PIN, INPUT_PULLUP);

  // initialize the motion sensor pin as an input:
  pinMode(MOTION_SENSOR_PIN, INPUT);

  // initialize the analog sensor pin:
  pinMode(ANALOG_SENSOR_PIN, INPUT);

  // attach interrupt to motion sensor pin
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), motionInterrupt, CHANGE);

  // attach interrupt to soil moisture sensor pin
  attachInterrupt(digitalPinToInterrupt(ANALOG_SENSOR_PIN), soilMoistureInterrupt, CHANGE);

  // initialize serial communication:
  Serial.begin(9600);
}

void loop() {
  // read the state of the PIR pushbutton value:
  int pirButtonState = digitalRead(PIR_BUTTON_PIN);

  // read the state of the soil moisture pushbutton value:
  int soilMoistureButtonState = digitalRead(SOIL_MOISTURE_BUTTON_PIN);

  // control LED according to the state of PIR button or motion sensor
  if (pirButtonState == LOW || motionSensorState == HIGH || soilMoistureButtonState == LOW) {
    // If PIR button is pressing, motion is detected, or soil moisture button is pressed
    digitalWrite(LED_PIN, HIGH); // turn on LED
    Serial.println("Motion detected ");

    // Disable sensors
    sensorsEnabled = false;
  } else {
    // otherwise, no button is pressing and no motion is detected
    digitalWrite(LED_PIN, LOW);  // turn off LED
    Serial.println("No motion Deatacted ");

    // Enable sensors
    sensorsEnabled = true;
  }

  if (sensorsEnabled) {
    // Variable to store ADC value (0 to 1023)
    int level;
    // analogRead function returns the integer 10-bit value (0 to 1023)
    level = analogRead(ANALOG_SENSOR_PIN);

    // Print text in serial monitor
    Serial.print("Analog value: ");
    // Print output voltage in serial monitor
    Serial.println(level);

    
  }

  delay(1000); // Optional delay to slow down the loop (adjust as needed)
}

// Motion Sensor Interrupt Service Routine
void motionInterrupt() {
  motionSensorState = digitalRead(MOTION_SENSOR_PIN);
}

// Soil Moisture Sensor Interrupt Service Routine
void soilMoistureInterrupt() {
  soilMoistureState = digitalRead(ANALOG_SENSOR_PIN);
}
