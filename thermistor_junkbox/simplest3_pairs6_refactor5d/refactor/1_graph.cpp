/**
 * Temperature, Voltage, and Current Monitoring System
 * 
 * This code handles data collection, history management, and visualization
 * on a TFT display for temperature sensors, voltage, and current measurements.
 */

// ======= Data History Management =======

/**
 * Update the history arrays for all measurements
 * 
 * @param temp1 Temperature from sensor 1 (°C)
 * @param temp2 Temperature from sensor 2 (°C)
 * @param tempDiff Temperature difference (°C)
 * @param voltage Measured voltage (V)
 * @param current Measured current (A)
 */
void updateDataHistory(double temp1, double temp2, double tempDiff, float voltage, float current) {
    // Shift all values to make room for new data
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        temp1_values[i] = temp1_values[i + 1];
        temp2_values[i] = temp2_values[i + 1];
        diff_values[i] = diff_values[i + 1];
        voltage_values[i] = voltage_values[i + 1];
        current_values[i] = current_values[i + 1];
    }
    
    // Add new readings at the end of each array
    int lastIndex = PLOT_WIDTH - 1;
    temp1_values[lastIndex] = temp1;
    temp2_values[lastIndex] = temp2;
    diff_values[lastIndex] = tempDiff;
    voltage_values[lastIndex] = voltage;
    current_values[lastIndex] = current;
}

// ======= Display Functions =======

/**
 * Clear the plot area and draw the zero reference line
 */
void preparePlotArea() {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);
    tft.drawFastHLine(PLOT_X_START, PLOT_Y_START + (PLOT_HEIGHT / 2), PLOT_WIDTH, PLOT_ZERO_COLOR);
}

/**
 * Draw a single data line on the plot
 * 
 * @param values Array of data values to plot
 * @param minValue Minimum value for scaling
 * @param maxValue Maximum value for scaling
 * @param color Line color
 */
void drawDataLine(const float values[], float minValue, float maxValue, uint16_t color) {
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        if (!isnan(values[i]) && !isnan(values[i + 1])) {
            int y_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(values[i], minValue, maxValue, 0, PLOT_HEIGHT);
            int y_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(values[i + 1], minValue, maxValue, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_prev, PLOT_X_START + i + 1, y_current, color);
        }
    }
}

/**
 * Plot all measurement data on the TFT display
 */
void plotAllData() {
    drawDataLine(temp1_values, MIN_TEMP, MAX_TEMP, GRAPH_COLOR_1);
    drawDataLine(temp2_values, MIN_TEMP, MAX_TEMP, GRAPH_COLOR_2);
    drawDataLine(diff_values, MIN_DIFF_TEMP, MAX_DIFF_TEMP, GRAPH_COLOR_DIFF);
    drawDataLine(voltage_values, MIN_VOLTAGE, MAX_VOLTAGE, GRAPH_COLOR_VOLTAGE);
    drawDataLine(current_values, MIN_CURRENT, MAX_CURRENT, GRAPH_COLOR_CURRENT);
}

/**
 * Display a labeled measurement value with optional color indicator
 * 
 * @param x X-coordinate for text
 * @param y Y-coordinate for text
 * @param label Label text
 * @param value Measurement value
 * @param units Units text
 * @param format Printf format string
 * @param color Color indicator (optional)
 * @param colorLabel Color indicator label (optional)
 */
void displayMeasurement(int x, int y, const char* label, float value, const char* units, 
                      const char* format, uint16_t color = 0, const char* colorLabel = NULL) {
    tft.setCursor(x, y);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(label);
    
    if (!isnan(value)) {
        tft.printf(format, value);
        tft.print(" ");
        tft.print(units);
    } else {
        tft.print("Error");
    }
    
    // Add color indicator if provided
    if (color != 0 && colorLabel != NULL) {
        tft.setTextColor(color, TFT_BLACK);
        tft.print(" ");
        tft.print(colorLabel);
    }
}

/**
 * Display all measurement labels and values
 * 
 * @param temp1 Temperature from sensor 1 (°C)
 * @param temp2 Temperature from sensor 2 (°C)
 * @param tempDiff Temperature difference (°C)
 * @param t1_millivolts Raw millivolts from sensor 1
 * @param voltage Measured voltage (V)
 * @param current Measured current (A)
 */
void displayAllLabels(double temp1, double temp2, double tempDiff, 
                    float t1_millivolts, float voltage, float current) {
    tft.setTextSize(LABEL_TEXT_SIZE);
    int line_height = 8;
    int col1_x = PLOT_X_START;
    int col2_x = PLOT_X_START + 100;
    int col3_x = PLOT_X_START + 260;
    int row1_y = LABEL_Y_START;
    int row2_y = LABEL_Y_START + line_height;
    int row3_y = LABEL_Y_START + 2 * line_height;
    
    // Column 1
    displayMeasurement(col1_x, row1_y, "T1: ", temp1, "C", "%.2f", GRAPH_COLOR_1, "R");
    displayMeasurement(col1_x, row2_y, "T2: ", temp2, "C", "%.2f", GRAPH_COLOR_2, "G");
    displayMeasurement(col1_x, row3_y, "dT: ", tempDiff, "C", "%.2f", GRAPH_COLOR_DIFF, "B");
    
    // Column 2
    displayMeasurement(col2_x, row1_y, "V: ", voltage, "V", "%.3f", GRAPH_COLOR_VOLTAGE, "Y");
    displayMeasurement(col2_x, row2_y, "I: ", current, "A", "%.3f", GRAPH_COLOR_CURRENT, "M");
    
    // Column 3
    displayMeasurement(col3_x, row1_y, "VCC:", thermistorSensor.getVCC(), "mV", "%.2f");
    displayMeasurement(col3_x, row2_y, "mV :", t1_millivolts, "mV", "%.2f");
    displayMeasurement(col3_x, row3_y, "%  :", dutyCycle, "", "%u");
}

// ======= Remote Control Handling =======

/**
 * Handle IR remote control commands
 */
void handleIRCommand() {
    if (IrReceiver.decodedIRData.protocol == SAMSUNG && IrReceiver.decodedIRData.address == 0x7) {
        // Debug output
        Serial.print(F("Command 0x"));
        Serial.println(IrReceiver.decodedIRData.command, HEX);
        
        // Process command
        switch(IrReceiver.decodedIRData.command) {
            case RemoteKeys::KEY_PLAY:
                isMeasuringResistance = true; 
                measureInternalResistance();
                break;
                
            case RemoteKeys::KEY_INFO:
                if (!isMeasuringResistance) {
                    displayInternalResistanceGraph();
                    delay(10000); // Wait 10 seconds
                }
                break;
        }
    }
}

// ======= Main Loop =======

/**
 * Main program loop
 */
void loop() {
    double temp1, temp2, tempDiff;
    float t1_millivolts;
    float voltage;
    float current;

    // Get sensor readings
    getThermistorReadings(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
    
    // Output readings to serial port
    printThermistorSerial(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
    
    // Update history and display
    updateDataHistory(temp1, temp2, tempDiff, voltage, current);
    preparePlotArea();
    plotAllData();
    displayAllLabels(temp1, temp2, tempDiff, t1_millivolts, voltage, current);

    // Uncomment to enable PWM control
    // controlPWM();

    // Uncomment to enable resistance display when not measuring
    // if (!isMeasuringResistance) {
    //     displayInternalResistanceGraph();
    // }

    // Handle IR remote commands
    if (IrReceiver.decode()) {
        handleIRCommand();
        IrReceiver.resume();
    }

    delay(500); // Reduced delay for more responsive PWM
}
