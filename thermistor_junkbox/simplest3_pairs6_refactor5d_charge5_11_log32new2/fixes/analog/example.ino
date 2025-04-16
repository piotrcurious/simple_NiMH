void loop() {
    // Read voltage on GPIO 34 (ADC1_CH6) with 11dB attenuation, 64 samples average
    int32_t mv = analogReadMillivolts(34, ADC_ATTEN_DB_11, 64);
    if (mv != -1) {
        Serial.printf("Pin 34 Voltage: %d mV\n", mv);
    } else {
        Serial.println("Error reading pin 34");
    }

    // Read voltage on GPIO 35 (ADC1_CH7) in microvolts using median filter (5 samples)
    int32_t uv_median = analogReadMedian(35, ADC_ATTEN_DB_0, 5);
     if (uv_median != INT32_MIN) {
        Serial.printf("Pin 35 Median Voltage: %d uV\n", uv_median);
    } else {
        Serial.println("Error reading pin 35 (median)");
    }

    // Read voltage on GPIO 32 (ADC1_CH4) using EMA filter
    int32_t uv_ema = analogReadEMA(32, ADC_ATTEN_DB_6, 30, false); // alpha=30, no reset
    if (uv_ema != INT32_MIN) {
        Serial.printf("Pin 32 EMA Voltage: %d uV\n", uv_ema);
    } else {
         Serial.println("Error reading pin 32 (EMA)");
    }

    // Read extended stats for GPIO 33 (ADC1_CH5)
    int32_t stats_uv[4];
    int32_t mean_uv = analogReadExtended(33, ADC_ATTEN_DB_2_5, 128, stats_uv);
    if (mean_uv != INT32_MIN) {
         Serial.printf("Pin 33 Stats (uV): Mean=%d, Min=%d, Max=%d, StdDev(scaled)=%d\n",
             stats_uv[2], stats_uv[0], stats_uv[1], stats_uv[3]);
    } else {
         Serial.println("Error reading pin 33 (extended)");
    }


    // Read internal temperature (if initialized)
    int16_t temp_c100 = readInternalTemperature(8); // Average 8 readings
    if (temp_c100 != INT16_MIN) {
        Serial.printf("Internal Temp: %.2f C\n", temp_c100 / 100.0f);
    } else {
        // Error already printed by the function usually
    }

    Serial.printf("Estimated Vref: %d mV\n", getADCVref());


    delay(2000);
}
