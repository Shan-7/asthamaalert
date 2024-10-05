
#include "DHT.h"

#define DHTPIN_OUTSIDE 4         // Pin connected to the outside DHT sensor
#define DHTPIN_NOSE 16           // Pin connected to the nose DHT sensor
#define DHTTYPE DHT11            // DHT11 or DHT22
#define MIN_INTERVAL 400         // Delay between sensor readings (in ms)
#define TEMP_THRESHOLD 0.35      // Threshold for temperature difference (in °C) to detect exhalation

DHT dhtOutside(DHTPIN_OUTSIDE, DHTTYPE);
DHT dhtNose(DHTPIN_NOSE, DHTTYPE);

unsigned long lastReadTime = 0;
int breathCount = 0;               // Count the number of breaths (exhale only)
bool exhaleDetected = false;       // Track if exhale has been detected

void setup() {
  Serial.begin(115200);
  dhtOutside.begin();
  dhtNose.begin();
}

void loop() {
  // Get the current time in milliseconds
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

  // Other code can run here without being blocked by delay()
}
