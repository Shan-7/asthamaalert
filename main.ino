// Include the necessary library files
#define BLYNK_TEMPLATE_ID "TMPL3k0wrxyuY"
#define BLYNK_TEMPLATE_NAME "Asthama Attack"
#define BLYNK_AUTH_TOKEN "Dx0s8e0vcphLVxx2tvDlImp9vUWuh9CP"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

char auth[] = "Dx0s8e0vcphLVxx2tvDlImp9vUWuh9CP";
char ssid[] = "Galaxy S";
char pass[] = "Frequency";

#define DHTPIN 21          // Pin where the DHT11 is connected
#define DHTTYPE DHT11     // DHT 11
#define BUZZER_PIN 15     // Pin for the passive buzzer

DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

// Threshold values for asthma triggers
float highTempThreshold = 35.0;  // High temperature trigger (30°C and above)
float lowTempThreshold = 18.0;   // Low temperature trigger (15°C and below)
float highHumidityThreshold = 75.0;  // High humidity trigger (above 70%)
float lowHumidityThreshold = 45.0;   // Low humidity trigger (below 30%)

void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);
  dht.begin();
  
  pinMode(BUZZER_PIN, OUTPUT); // Initialize the buzzer pin

  // Setup a function to be called every second
  timer.setInterval(1000L, sendSensorData);
}

void sendSensorData() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println("°C");

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.println("%");

  // Send temperature and humidity values to Blynk
  Blynk.virtualWrite(V2, t);
  Blynk.virtualWrite(V3, h);

  // High temperature or high humidity alert
  if (t > highTempThreshold || h > highHumidityThreshold) {
    Blynk.logEvent("asthma_high_temp_hum_alert", "High Temperature or Humidity Alert!");
    Serial.println("Asthma Alert: High temperature or humidity!");
    buzzBuzzer();  // Call the buzzer function
  }

  // Low temperature alert
  if (t < lowTempThreshold) {
    Blynk.logEvent("asthma_low_temp_alert", "Low Temperature Alert!");
    Serial.println("Asthma Alert: Low temperature!");
    buzzBuzzer();  // Call the buzzer function
  }

  // Low humidity alert (dry air)
  if (h < lowHumidityThreshold) {
    Blynk.logEvent("asthma_low_humidity_alert", "Low Humidity Alert!");
    Serial.println("Asthma Alert: Low humidity (dry air)!");
    buzzBuzzer();  // Call the buzzer function
  }
}

void buzzBuzzer() {
  tone(BUZZER_PIN, 1000); // Send a 1kHz tone to the buzzer
  delay(5000);            // Keep the tone on for 5 seconds
  noTone(BUZZER_PIN);     // Stop the tone
}

void loop() {
  Blynk.run();
  timer.run();
}
