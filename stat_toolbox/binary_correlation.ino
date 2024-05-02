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

// Binary search to find the segment with the highest correlation
int binarySearchHighCorrelation(int data1[], int data2[], int size, float threshold) {
  int left = 0;
  int right = size - 1;
  int maxIndex = -1;
  float maxCorrelation = 0;
  
  while (left <= right) {
    int mid = left + (right - left) / 2;
    float correlation = pearsonCorrelation(data1, data2 + mid, size - mid);
    
    if (correlation > threshold && correlation > maxCorrelation) {
      maxCorrelation = correlation;
      maxIndex = mid;
    }
    
    if (correlation < threshold) {
      left = mid + 1;
    } else {
      right = mid - 1;
    }
  }
  
  return maxIndex;
}

void setup() {
  Serial.begin(115200);
  
  // Example data arrays from two sensors
  int sensorData1[] = { /* Your sensor data here */ };
  int sensorData2[] = { /* Your sensor data here */ };
  int dataSize = sizeof(sensorData1) / sizeof(sensorData1[0]);
  
  // Define the correlation threshold
  float correlationThreshold = 0.8;
  
  // Find the segment with the highest correlation
  int index = binarySearchHighCorrelation(sensorData1, sensorData2, dataSize, correlationThreshold);
  
  if (index != -1) {
    Serial.print("Highly correlated segment starts at index: ");
    Serial.println(index);
  } else {
    Serial.println("No highly correlated segments found.");
  }
}

void loop() {
  // Your code here
}
