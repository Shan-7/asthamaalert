#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "DHT.h"

// Define MAX30105 Sensor
MAX30105 particleSensor;

// Define DHT Sensors
#define DHTPIN_OUTSIDE 4         // Pin connected to the outside DHT sensor
#define DHTPIN_NOSE 16           // Pin connected to the nose DHT sensor
#define DHTTYPE DHT11            // DHT11 or DHT22

DHT dhtOutside(DHTPIN_OUTSIDE, DHTTYPE);
DHT dhtNose(DHTPIN_NOSE, DHTTYPE);

// Define LM35 Sensor
const int LM35_PIN = 34;

// Dust Sensor Settings
int measurePin = 32;
int ledPower = 15;
unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 280;//9680

// SpO2 and Heart Rate Variables
#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100]; // Infrared LED sensor data
uint32_t redBuffer[100]; // Red LED sensor data
int32_t bufferLength; // Data length
int32_t spo2; // SpO2 value
int8_t validSPO2; // Indicator to show if the SpO2 calculation is valid
int32_t heartRate; // Heart rate value
int8_t validHeartRate; // Indicator to show if the heart rate calculation is valid

// Breathing Detection Variables
unsigned long lastReadTime = 0;
int breathCount = 0;               // Count the number of breaths (exhale only)
bool exhaleDetected = false;       // Track if exhale has been detected
#define MIN_INTERVAL 100 //400        // Delay between sensor readings (in ms)
#define TEMP_THRESHOLD 0.5      // Threshold for temperature difference (in °C) to detect exhalation

void setup() {
  Serial.begin(115200);

  // Initialize the MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 sensor not found. Please check wiring/power.");
    while (1);
  }

  // Initialize the DHT sensors
  dhtOutside.begin();
  dhtNose.begin();

  // Set up dust sensor
  pinMode(ledPower, OUTPUT);
  
  // Setup MAX30105 sensor parameters
  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;
  byte sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure MAX30105 sensor
}

void loop() {
  // Read heart rate and SpO2
  readHeartRateAndSpO2();

  // Read dust density
  float dustDensity = readDustDensity();

  // Read temperatures and detect breath count
  readTemperatureAndBreathCount();

  // Read LM35 temperature
  double lm35Temperature = readLM35Temperature();

  // Print all sensor data
  Serial.print("Heart Rate: ");
  Serial.print(heartRate);
  Serial.print(", SpO2: ");
  Serial.print(spo2);
  Serial.print(", Breath Count: ");
  Serial.print(breathCount);
  Serial.print(", Dust Density: ");
  Serial.print(dustDensity);
  Serial.print(", Microphone: ");
  Serial.println(lm35Temperature);

  delay(100); // Delay for readability 400
}

// Function to read heart rate and SpO2
void readHeartRateAndSpO2() {
  bufferLength = 100; // Buffer length for MAX30105

  // Read samples from MAX30105 sensor
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) {
      particleSensor.check(); // Check the sensor for new data
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // Move to next sample
  }

  // Calculate heart rate and SpO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

// Function to read dust density
float readDustDensity() {
  digitalWrite(ledPower, LOW);
  delayMicroseconds(samplingTime);
  int voMeasured = analogRead(measurePin);
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH);
  delayMicroseconds(sleepTime);
  
  float calcVoltage = voMeasured * (5.0 / 1024);
  float dustDensity = 0.17 * calcVoltage - 0.1;

  if (dustDensity < 0) {
    dustDensity = 0.00;
  }

  return dustDensity;
}

// Function to read temperatures and detect breath count
void readTemperatureAndBreathCount() {
  unsigned long currentTime = millis();

  // Only read sensor if minimum interval has passed
  if (currentTime - lastReadTime >= MIN_INTERVAL) {
    lastReadTime = currentTime;

    // Reading temperatures from both sensors
    float tempOutside = dhtOutside.readTemperature();
    float tempNose = dhtNose.readTemperature();

    // Check if the readings are valid
    if (isnan(tempOutside) || isnan(tempNose)) {
      Serial.println("Failed to read from one of the DHT sensors!");
    } else {
      // Calculate the temperature difference
      float tempDifference = tempNose - tempOutside;

      // Print temperatures and the temperature difference
      Serial.print("Outside Temp: ");
      Serial.print(tempOutside);
      Serial.print(" *C\t");
      Serial.print("Nose Temp: ");
      Serial.print(tempNose);
      Serial.print(" *C\t");
      Serial.print("Temp Difference: ");
      Serial.print(tempDifference);
      Serial.println(" *C");

      // Detect exhalation (temperature difference exceeds threshold)
      if (tempDifference > TEMP_THRESHOLD && !exhaleDetected) {
        exhaleDetected = true;  // Mark exhale detected
        breathCount++;          // Increment breath count
        Serial.print("Exhale detected. Breath count: ");
        Serial.println(breathCount);
      }

      // Detect inhale (temperature difference drops below the threshold)
      if (tempDifference <= TEMP_THRESHOLD && exhaleDetected) {
        exhaleDetected = false;  // Reset for the next exhale detection
        Serial.println("Inhale detected, ready for next exhale.");
      }
    }
  }
}

// Function to read LM35 temperature
double readLM35Temperature() {
  int value1 = analogRead(LM35_PIN);
  double tempC = value1 * (5.0 / 1023.0) * 100; // Convert to Celsius
  return tempC;
}
