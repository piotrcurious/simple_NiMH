
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
  
  ambientTemp = adt75.readTemperature(); // Read ambient temperature from adt75A thermometer
  
  measureBatteryTemp(); // Measure battery temperature of each channel
  
  calculateBatteryDT(); // Calculate battery temperature difference of each channel
  
  updateBufferDT(); // Update rolling buffer of temperature difference of each channel
  
  checkChargingStatus(); // Check charging status of each channel
  
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
    /* Convert pulse count to temperature using a function that depends on the 
       characteristics of the thermistor and the capacitor */
    
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


