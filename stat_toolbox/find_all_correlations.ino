#include <math.h>

// Function to calculate the Pearson correlation coefficient
float pearsonCorrelation(int data1[], int data2[], int size) {
  float sumX = 0, sumY = 0, sumX2 = 0, sumY2 = 0, sumXY = 0;
  for (int i = 0; i < size; i++) {
    sumX += data1[i];
    sumY += data2[i];
    sumX2 += pow(data1[i], 2);
    sumY2 += pow(data2[i], 2);
    sumXY += data1[i] * data2[i];
  }
  
  float numerator = size * sumXY - sumX * sumY;
  float denominator = sqrt((size * sumX2 - pow(sumX, 2)) * (size * sumY2 - pow(sumY, 2)));
  
  if (denominator == 0) {
    return 0;
  }
  
  return numerator / denominator;
}

// Function to find all highly correlated segments
void findAllHighCorrelationSegments(int data1[], int data2[], int size, float threshold) {
  for (int start = 0; start < size; start++) {
    for (int end = start + 1; end <= size; end++) {
      float correlation = pearsonCorrelation(data1 + start, data2 + start, end - start);
      if (correlation >= threshold) {
        Serial.print("Highly correlated segment found from index ");
        Serial.print(start);
        Serial.print(" to ");
        Serial.print(end - 1);
        Serial.print(" with length ");
        Serial.print(end - start);
        Serial.print(" and correlation ");
        Serial.println(correlation, 4); // Print correlation to 4 decimal places
        start = end; // Move to the next segment
        break; // Exit the inner loop
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Example data arrays from two sensors
  int sensorData1[] = { /* Your sensor data here */ };
  int sensorData2[] = { /* Your sensor data here */ };
  int dataSize = sizeof(sensorData1) / sizeof(sensorData1[0]);
  
  // Define the correlation threshold
  float correlationThreshold = 0.8;
  
  // Find all highly correlated segments
  findAllHighCorrelationSegments(sensorData1, sensorData2, dataSize, correlationThreshold);
}

void loop() {
  // Your code here
}
