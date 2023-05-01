
// Arduino code for four channel Ni-MH battery charger
// Using dT method to end charge
// Using fixed 200mA current limiting circuit for each channel
// Using adt75A thermometer for ambient temperature
// Using NTC thermistors for battery temperature
// Using 10uF capacitors charged by NTC thermistors
// To boost discharging, connect a diode across thermistor
// so it discharges capacitor when the pin is low. 

#include <Wire.h> // Library for I2C communication
#include <ADT75.h> // Library for adt75A thermometer

#define CH1_PIN 2 // Digital pin for channel 1 switch
#define CH2_PIN 3 // Digital pin for channel 2 switch
#define CH3_PIN 4 // Digital pin for channel 3 switch
#define CH4_PIN 5 // Digital pin for channel 4 switch

#define CH1_THERM A0 // Analog pin for channel 1 thermistor
#define CH2_THERM A1 // Analog pin for channel 2 thermistor
#define CH3_THERM A2 // Analog pin for channel 3 thermistor
#define CH4_THERM A3 // Analog pin for channel 4 thermistor

#define BUFFER_SIZE 10 // Size of rolling buffer array
#define DT_THRESHOLD 1 // Temperature difference threshold in Celsius

ADT75 adt75; // Create an instance of adt75A thermometer

float ambientTemp; // Variable to store ambient temperature
float batteryTemp[4]; // Array to store battery temperature of each channel
float batteryDT[4]; // Array to store battery temperature difference of each channel
int bufferIndex[4]; // Array to store current index of rolling buffer array
float bufferDT[4][BUFFER_SIZE]; // Array to store rolling buffer of temperature difference of each channel
bool charging[4]; // Array to store charging status of each channel

void setup() {
  Serial.begin(9600); // Start serial communication
  Wire.begin(); // Start I2C communication
  adt75.begin(); // Start adt75A thermometer
  
  pinMode(CH1_PIN, OUTPUT); // Set channel 1 pin as output
  pinMode(CH2_PIN, OUTPUT); // Set channel 2 pin as output
  pinMode(CH3_PIN, OUTPUT); // Set channel 3 pin as output
  pinMode(CH4_PIN, OUTPUT); // Set channel 4 pin as output
  
  digitalWrite(CH1_PIN, HIGH); // Turn on channel 1 switch
  digitalWrite(CH2_PIN, HIGH); // Turn on channel 2 switch
  digitalWrite(CH3_PIN, HIGH); // Turn on channel 3 switch
  digitalWrite(CH4_PIN, HIGH); // Turn on channel 4 switch
  
  for (int i = 0; i < 4; i++) { // Initialize variables and arrays
    batteryTemp[i] = 0;
    batteryDT[i] = 0;
    bufferIndex[i] = -1;
    charging[i] = true;
    for (int j = 0; j < BUFFER_SIZE; j++) {
      bufferDT[i][j] = -100; // Set a large negative value as initial value
    }
  }
}

void loop() {
  
  //ambientTemp = adt75.readTemperature(); // Read ambient temperature from adt75A thermometer
  ambientTemp = adt75.Measure_Temp(); // depends on library used
  measureBatteryTemp(); // Measure battery temperature of each channel
  
  calculateBatteryDT(); // Calculate battery temperature difference of each channel

  static unsigned long prev_time = 0; // Previous time when data was sent (milliseconds)

  // Check if the current time is more than 1 minute after the previous time
  if (millis() - prev_time > 1 * 60 * 1000) {
    // If yes, update the previous time and update the buffer 
    prev_time = millis();
  updateBufferDT(); // Update rolling buffer of temperature difference of each channel
  }

// Check if the current time is more than BUFFER_SIZE+1 minutes since startup 
// this ensures buffer is filled with data
// and temperatures stabilized. 
   if (millis()  > BUFFER_SIZE+1 * 60 * 1000) {
    checkChargingStatus(); // Check charging status of each channel
   }
}

/*

// method using external pullup to charge capacitor
// much slower, but you can use trimpot to tune the
// pullup and is less variable on Arduino supply voltage noise

void measureBatteryTemp() {
  
  int pulseCount[4]; // Array to store pulse count of each thermistor
  
  for (int i = CH1_THERM; i <= CH4_THERM; i++) { // Loop through each analog pin
    
    pulseCount[i - CH1_THERM] = -1; // Initialize pulse count as -1
    
    pinMode(i, OUTPUT); // Set analog pin as output
    
    digitalWrite(i, LOW); // Discharge capacitor
    
    delay(100); // Wait for capacitor to discharge
    
    pinMode(i, INPUT); // Set analog pin as input
    
    while (digitalRead(i) == LOW) { // Count pulses until capacitor is charged
      
      pulseCount[i - CH1_THERM]++; 
      
    }
    
    batteryTemp[i - CH1_THERM] = pulseCountToTemp(pulseCount[i - CH1_THERM]); 
    // Convert pulse count to temperature using a function that depends on the 
       characteristics of the thermistor and the capacitor 
    
    Serial.print("Channel ");
    Serial.print(i - CH1_THERM + 1);
    Serial.print(" battery temperature: ");
    Serial.print(batteryTemp[i - CH1_THERM]);
    Serial.println(" C");
    
  }
  
}
*/

// integrating method, less prone to interrupt noise and with failure mode

void measureBatteryTemp() {
  
  int pulseCount[4]; // Array to store pulse count of each thermistor
  
  for (int i = CH1_THERM; i <= CH4_THERM; i++) { // Loop through each analog pin
    
    pulseCount[i - CH1_THERM] = 0; // Initialize pulse count as 0
    
    pinMode(i, OUTPUT); // Set analog pin as output
    
    digitalWrite(i, LOW); // Discharge capacitor
    
    delay(100); // Wait for capacitor to discharge
    
    unsigned long startTime = millis(); // Get current time in milliseconds
    
    while (millis() - startTime < 1000) { // Charge capacitor until timeout
      
      pinMode(i, OUTPUT); // Set analog pin as output
      
      digitalWrite(i, HIGH); // Send a charging pulse
      
      delay(10); // Wait for capacitor to charge
      
      pulseCount[i - CH1_THERM]++; // Increment pulse count
      
      pinMode(i, INPUT); // Set analog pin as input
      
      if (digitalRead(i) == HIGH) { // If pin reads HIGH
        
        break; // Break the loop
        
      }
      
    }
    
    if (pulseCount[i - CH1_THERM] == 0) { // If no pulses were counted
      
      batteryTemp[i - CH1_THERM] = -100; // Set a large negative value as temperature
      
    } else { // If pulses were counted
      
      batteryTemp[i - CH1_THERM] = pulseCountToTemp(pulseCount[i - CH1_THERM]); 
      /* Convert pulse count to temperature using a function that depends on the 
         characteristics of the thermistor and the capacitor */
      
    }
    
    Serial.print("Channel ");
    Serial.print(i - CH1_THERM + 1);
    Serial.print(" battery temperature: ");
    Serial.print(batteryTemp[i - CH1_THERM]);
    Serial.println(" C");
    
  }
  
}

float pulseCountToTemp(int pulseCount) {
  
  float resistance; // Variable to store thermistor resistance
  float temperature; // Variable to store thermistor temperature
  
  float pulseLength = 10; // Variable to store pulse length in milliseconds
  float capacitorValue = 10e-6; // Variable to store capacitor value in farads
  float analogVoltage = 5; // Variable to store analog pin voltage in volts
  
  resistance = (analogVoltage * capacitorValue * pulseCount * 1000) / (pulseLength * log(2)); // Calculate resistance from pulse count using Kirchhoff's law
  
  temperature = 1 / (log(resistance / 10000) / 3950 + 1 / 298.15) - 273.15; // Calculate temperature from resistance using a formula that depends on the thermistor parameters
  
  return temperature; // Return temperature
  
}

void calculateBatteryDT() {
  
  for (int i = 0; i < 4; i++) { // Loop through each channel
    
    batteryDT[i] = batteryTemp[i] - ambientTemp; // Calculate battery temperature difference
    
    Serial.print("Channel ");
    Serial.print(i + 1);
    Serial.print(" battery temperature difference: ");
    Serial.print(batteryDT[i]);
    Serial.println(" C");
    
  }
  
}


void updateBufferDT() {
  
  for (int i = 0; i < 4; i++) { // Loop through each channel
    
    bufferIndex[i] = (bufferIndex[i] + 1) % BUFFER_SIZE; // Update buffer index in a circular way
    
    bufferDT[i][bufferIndex[i]] = batteryDT[i]; // Store battery temperature difference in buffer
    
    Serial.print("Channel ");
    Serial.print(i + 1);
    Serial.print(" buffer: ");
    
    for (int j = 0; j < BUFFER_SIZE; j++) { // Print buffer values
      
      Serial.print(bufferDT[i][j]);
      Serial.print(" ");
      
    }
    
    Serial.println();
    
  }
  
}

/* simple rising trend detection. it will fail. 
void checkChargingStatus() {
  
  for (int i = 0; i < 4; i++) { // Loop through each channel
    
    if (charging[i]) { // If channel is charging
      
      bool risingTrend = true; // Variable to store if there is a rising trend in buffer
      
      for (int j = 0; j < BUFFER_SIZE - 1; j++) { // Loop through buffer values
        
        int index1 = (bufferIndex[i] + j) % BUFFER_SIZE; // Get first index
        int index2 = (bufferIndex[i] + j + 1) % BUFFER_SIZE; // Get second index
        
        if (bufferDT[i][index2] <= bufferDT[i][index1]) { // If second value is not greater than first value
          
          risingTrend = false; // Set rising trend to false
          break; // Break the loop
          
        }
        
      }
      
      if (risingTrend && batteryDT[i] >= DT_THRESHOLD) { // If there is a rising trend and battery temperature difference is above threshold
        
        charging[i] = false; // Set charging status to false
        
        switch (i) { // Turn off channel switch
          
          case 0:
            digitalWrite(CH1_PIN, LOW);
            break;
          case 1:
            digitalWrite(CH2_PIN, LOW);
            break;
          case 2:
            digitalWrite(CH3_PIN, LOW);
            break;
          case 3:
            digitalWrite(CH4_PIN, LOW);
            break;
            
        }
        
        Serial.print("Channel ");
        Serial.print(i + 1);
        Serial.println(" charging stopped.");
        
      }
      
    }
    
  }
  
}
*/

void checkChargingStatus() {
  
  for (int i = 0; i < 4; i++) { // Loop through each channel
    
    if (charging[i]) { // If channel is charging
      
      float bufferSum = 0; // Variable to store the sum of buffer values
      
      for (int j = 0; j < BUFFER_SIZE; j++) { // Loop through buffer values
        
        bufferSum += bufferDT[i][j]; // Add buffer value to sum
        
      }
      
      if (bufferSum >= DT_THRESHOLD * BUFFER_SIZE) { // If the average of buffer values is above threshold
        
        charging[i] = false; // Set charging status to false
        
        switch (i) { // Turn off channel switch
          
          case 0:
            digitalWrite(CH1_PIN, LOW);
            break;
          case 1:
            digitalWrite(CH2_PIN, LOW);
            break;
          case 2:
            digitalWrite(CH3_PIN, LOW);
            break;
          case 3:
            digitalWrite(CH4_PIN, LOW);
            break;
            
        }
        
        Serial.print("Channel ");
        Serial.print(i + 1);
        Serial.println(" charging stopped.");
        
      }
      
    }
    
  }
  
}
 
