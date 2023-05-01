
// Arduino code for realtime visualization of charging process
// Using serial input data from Arduino
// Using fabgl for VGA output on ESP32

#include <fabgl.h> // Library for fabgl graphics

#define RX_PIN 16 // RX pin for serial communication
#define TX_PIN 17 // TX pin for serial communication

DisplayController display; // Create a display controller object
Canvas canvas(&display); // Create a canvas object

float ambientTemp; // Variable to store ambient temperature
float batteryDT[4]; // Array to store battery temperature difference of each channel
int xdata = 0; // Variable to store x-axis data
float totalmAh[4]; // Array to store total mAh of each channel
bool chargingStatus[4]; // Array to store charging status of each channel

Color colors[5] = {Color::White, Color::Red, Color::Green, Color::Blue, Color::Yellow}; // Array of colors for the graph

void setup() {
  
  Serial.begin(9600); // Start serial communication with PC
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Start serial communication with Arduino
  
  display.begin(); // Initialize the display controller
  display.setResolution(VGA_640x480_60Hz); // Set the resolution to 640x480 at 60Hz
  
  canvas.setBrushColor(Color::Black); // Set the brush color to black
  canvas.clear(); // Clear the canvas
  
  canvas.setPenColor(Color::White); // Set the pen color to white
  canvas.drawText(10, 10, "Realtime Visualization of Charging Process"); // Draw the title
  
  canvas.drawLine(50, 50, 50, 430); // Draw the y-axis
  canvas.drawLine(50, 430, 590, 430); // Draw the x-axis
  
  canvas.drawText(20, 40, "Temp"); // Draw the y-axis label
  canvas.drawText(600, 420, "Time"); // Draw the x-axis label
  
}

void loop() {
  
  if (Serial2.available()) { // If there is data from Arduino
    
    String line = Serial2.readStringUntil('\n'); // Read a line from Arduino
    
    if (line.startsWith("Channel")) { // If line contains temperature data
      
      int channel = line.substring(8, 9).toInt() - 1; // Get the channel number
      
      float temp = line.substring(26).toFloat(); // Get the temperature value
      
      if (channel == 0) { // If channel is 1
        
        ambientTemp = temp; // Store temperature as ambient temperature
        
        xdata++; // Increment x-axis data
        
      } else { // If channel is not 1
        
        batteryDT[channel - 1] = temp; // Store temperature as battery temperature difference
        
      }
      
      if (chargingStatus[channel]) { // If channel is charging
        
        totalmAh[channel] += 0.2; // Add 200mA to total mAh
        
      }
      
      drawGraph(); // Draw the graph
      
    } else if (line.startsWith("Charging stopped")) { // If line contains charging status
      
      int channel = line.substring(9, 10).toInt() - 1; // Get the channel number
      
      chargingStatus[channel] = false; // Set charging status to false
      
     // drawLabel(channel); // Draw the label
      //not implemented
    }
    
  }
  
}

I'll try to finishI'll try to finish the drawGraph function. Here is the complete code:

```c++
void drawGraph() {
  
  canvas.setBrushColor(Color::Black); // Set the brush color to black
  canvas.fillRect(51, 51, xdata + 50, 429); // Clear the graph area
  
  for (int i = -1; i < 4; i++) { // Loop through each channel
    
    float temp; // Variable to store temperature
    
    if (i == -1) { // If channel is ambient
      
      temp = ambientTemp; // Get ambient temperature
      
    } else { // If channel is not ambient
      
      temp = batteryDT[i]; // Get battery temperature difference
      
    }
    
    int y = map(temp, -10, +10, 430, 50); // Map temperature to y coordinate
    
    canvas.setPenColor(colors[i + 1]); // Set the pen color
    
    if (xdata > 0) { // If there is previous data
      
      canvas.drawLine(xdata + 49, ydata[i + 1], xdata + 50, y); 
      /* Draw a line from previous point to current point */
      
    }
    
    ydata[i + 1] = y; // Store current y coordinate as previous
    
  }
  
}

void drawGraph() {
  
  canvas.setBrushColor(Color::Black); // Set the brush color to black
  canvas.fillRect(51, 51, xdata + 50, 429); // Clear the graph area
  
  for (int i = -1; i < 4; i++) { // Loop through each channel
    
    float temp; // Variable to store temperature
    
    if (i == -1) { // If channel is ambient
      
      temp = ambientTemp; // Get ambient temperature
      
    } else { // If channel is not ambient
      
      temp = batteryDT[i]; // Get battery temperature difference
      
    }
    
    int y = map(temp, 17, +30, 430, 50); // Map temperature to y coordinate
    
    canvas.setPenColor(colors[i + 1]); // Set the pen color
    
    if (xdata > 0) { // If there is previous data
      
      canvas.drawLine(xdata + 49, ydata[i + 1], xdata + 50, y); 
      /* Draw a line from previous point to current point */
      
    }
    
    ydata[i + 1] = y; // Store current y coordinate as previous
    
  }
  
}

