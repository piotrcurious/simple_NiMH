#include <math.h>

// Function to calculate mean
float calculateMean(float data[], int len) {
  float sum = 0.0;
  for (int i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum / len;
}

// Function to calculate Pearson correlation
float calculatePearson(float X[], float Y[], int len) {
  float xMean = calculateMean(X, len);
  float yMean = calculateMean(Y, len);
  
  float numerator = 0.0;
  float sumSqX = 0.0;
  float sumSqY = 0.0;
  
  for (int i = 0; i < len; i++) {
    numerator += (X[i] - xMean) * (Y[i] - yMean);
    sumSqX += pow(X[i] - xMean, 2);
    sumSqY += pow(Y[i] - yMean, 2);
  }
  
  return numerator / sqrt(sumSqX * sumSqY);
}

void setup() {
  Serial.begin(115200);
  
  // Example data arrays
  float dataX[] = {1, 2, 3, 4, 5};
  float dataY[] = {5, 4, 3, 2, 1};
  int dataLength = sizeof(dataX) / sizeof(dataX[0]);
  
  // Calculate Pearson correlation
  float pearsonCorrelation = calculatePearson(dataX, dataY, dataLength);
  
  // Print the result
  Serial.print("Pearson correlation coefficient: ");
  Serial.println(pearsonCorrelation, 6); // Print with 6 decimal places
}

void loop() {
  // Not used in this example
}
