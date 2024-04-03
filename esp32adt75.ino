#include <Wire.h>
#include <ADT75.h> // Install the library from the link provided

// Create an instance of the ADT75 sensor
ADT75 adt75;

// Global variables
float temperature; // Exposed temperature reading
float kalmanGain = 0.1; // Exposed Kalman gain (adjust as needed)

// Kalman filter variables
float estimatedTemp = 25.0; // Initial estimate
float estimatedError = 1.0; // Initial estimate error
float processNoise = 0.01; // Process noise (adjust as needed)

// Task handles
TaskHandle_t tempReadingTask;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  adt75.begin();

  // Create the temperature reading task
  xTaskCreatePinnedToCore(
    tempReadingTaskFunction, // Task function
    "TempReadingTask",      // Task name
    4096,                   // Stack size
    NULL,                   // Task parameters
    1,                      // Priority
    &tempReadingTask,       // Task handle
    0                       // Core (0 or 1)
  );
}

void loop() {
  // Other tasks or code can run here
  delay(1000); // Adjust as needed
}

void tempReadingTaskFunction(void* parameter) {
  while (1) {
    // Read raw temperature value from ADT75
    int16_t rawTemp = adt75.readTemperatureRaw();

    // Convert raw value to Celsius
    temperature = adt75.convertToCelsius(rawTemp);

    // Kalman filter update
    float kalmanGainFactor = estimatedError / (estimatedError + processNoise);
    estimatedTemp = estimatedTemp + kalmanGainFactor * (temperature - estimatedTemp);
    estimatedError = (1.0 - kalmanGainFactor) * estimatedError;

    // Print filtered temperature
    Serial.print("Filtered Temperature: ");
    Serial.println(estimatedTemp);

    // Delay before next reading
    vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust as needed
  }
}
