#include "analog.h"

adc_calibration_data_t adc1_cal_data[ADC1_CHANNEL_COUNT];
bool adc1_cal_initialized[ADC1_CHANNEL_COUNT] = {false};

/**
 * Read analog value in millivolts with improved numerical stability
 * @param pin GPIO pin number
 * @param attenuation ADC attenuation
 * @param oversampling Number of samples to average (use powers of 2 for efficient division)
 * @return Voltage in millivolts, or -1 on error
 */
int analogReadMillivolts(int pin, adc_atten_t attenuation, int oversampling) {
  int adc1_chan = get_adc1_channel(pin);
  if (adc1_chan == -1) {
    log_e("analogReadMillivolts", "Invalid pin for ADC1: %d", pin);
    return -1;
  }

  // Configure ADC
  adc1_config_width(ADC_WIDTH_BIT_12); // Using 12-bit resolution
  esp_err_t err = adc1_config_channel_atten((adc1_channel_t)adc1_chan, attenuation);
  if (err != ESP_OK) {
    log_e("analogReadMillivolts", "Error configuring attenuation for pin %d: %s", pin, esp_err_to_name(err));
    return -1;
  }

  // Initialize calibration data if not already initialized or if attenuation changed
  if (!adc1_cal_initialized[adc1_chan] || adc1_cal_data[adc1_chan].current_atten != attenuation) {
    if (adc1_cal_data[adc1_chan].adc_chars != NULL) {
      free(adc1_cal_data[adc1_chan].adc_chars);
    }
    adc1_cal_data[adc1_chan].adc_chars = (esp_adc_cal_characteristics_t *)malloc(sizeof(esp_adc_cal_characteristics_t));
    if (adc1_cal_data[adc1_chan].adc_chars == NULL) {
      log_e("analogReadMillivolts", "Memory allocation failed for calibration data");
      return -1;
    }
    esp_adc_cal_characterize(ADC_UNIT_1, attenuation, ADC_WIDTH_BIT_12, 3300, adc1_cal_data[adc1_chan].adc_chars); // Assuming 3.3V Vref
    adc1_cal_data[adc1_chan].current_atten = attenuation;
    adc1_cal_initialized[adc1_chan] = true;
  }

  // Handle oversampling using Kahan summation for better numerical stability
  uint32_t raw_adc = 0;
  if (oversampling > 1) {
    // Check if oversampling is a power of 2 for efficient division
    bool isPowerOf2 = (oversampling & (oversampling - 1)) == 0;
    int shift = 0;
    
    if (isPowerOf2) {
      // Calculate log2 for division by shifting
      int temp = oversampling;
      while (temp > 1) {
        temp >>= 1;
        shift++;
      }
    }
    
    // Use Kahan summation algorithm for numerical stability
    float sum = 0.0f;
    float compensation = 0.0f;  // Running compensation for lost low-order bits
    
    for (int i = 0; i < oversampling; i++) {
      uint32_t sample = adc1_get_raw((adc1_channel_t)adc1_chan);
      
      // Apply Kahan summation formula
      float y = static_cast<float>(sample) - compensation;
      float t = sum + y;
      compensation = (t - sum) - y;  // Compute the lost bits
      sum = t;
    }
    
    // Use bit shift for power-of-2 divisors (more efficient and exact)
    if (isPowerOf2) {
      raw_adc = static_cast<uint32_t>(sum) >> shift;
    } else {
      // Use proper rounding for non-power-of-2 divisors
      raw_adc = static_cast<uint32_t>(sum / oversampling + 0.5f);
    }
  } else {
    raw_adc = adc1_get_raw((adc1_channel_t)adc1_chan);
  }

  // Bound check to avoid potential overflow issues
  if (raw_adc > 4095) {
    raw_adc = 4095;  // Cap at max 12-bit value
  }

  uint32_t voltage = 0;
  if (adc1_cal_initialized[adc1_chan] && adc1_cal_data[adc1_chan].adc_chars != NULL) {
    // Use calibration data to convert raw value to voltage
    voltage = esp_adc_cal_raw_to_voltage(raw_adc, adc1_cal_data[adc1_chan].adc_chars);
    return static_cast<int>(voltage); // voltage is in mV
  } else {
    // If calibration failed, use a more robust conversion method
    log_w("analogReadMillivolts", "Calibration not initialized for pin %d, using approximation.", pin);
    
    // Use fixed-point arithmetic to avoid floating-point errors
    // Scale by 3300 first, then divide by 4095 to maintain precision
    // Adding half of divisor for proper rounding
    uint32_t scaled = (raw_adc * 3300UL + 2048UL) / 4095UL;
    return static_cast<int>(scaled);
  }
}
