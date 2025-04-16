#include "analog.h"

adc_calibration_data_t adc1_cal_data[ADC1_CHANNEL_COUNT];
bool adc1_cal_initialized[ADC1_CHANNEL_COUNT] = {false};

// Structure to hold Kahan summation state for integer operations
typedef struct {
    int64_t sum;        // Current sum
    int64_t correction; // Running compensation term
} kahan_sum_t;

/**
 * Initialize a Kahan summation structure
 * @param ksum Pointer to kahan_sum_t structure
 */
static inline void kahan_init(kahan_sum_t* ksum) {
    ksum->sum = 0;
    ksum->correction = 0;
}

/**
 * Add a value to Kahan sum using integer-only operations
 * @param ksum Pointer to kahan_sum_t structure
 * @param value Value to add
 */
static inline void kahan_add(kahan_sum_t* ksum, int32_t value) {
    int64_t y = value - ksum->correction;
    int64_t t = ksum->sum + y;
    ksum->correction = (t - ksum->sum) - y;
    ksum->sum = t;
}

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

  // Handle oversampling using integer-based Kahan summation for maximum stability
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
    
    // Use integer-based Kahan summation algorithm for numerical stability
    kahan_sum_t ksum;
    kahan_init(&ksum);
    
    for (int i = 0; i < oversampling; i++) {
      uint32_t sample = adc1_get_raw((adc1_channel_t)adc1_chan);
      kahan_add(&ksum, sample);
    }
    
    // Use bit shift for power-of-2 divisors (more efficient and exact)
    if (isPowerOf2) {
      raw_adc = (uint32_t)((ksum.sum + (1LL << (shift - 1))) >> shift); // Add half divisor for rounding
    } else {
      // Integer division with proper rounding
      raw_adc = (uint32_t)((ksum.sum + (oversampling >> 1)) / oversampling);
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

/**
 * Read analog value in microvolts with improved numerical stability
 * Uses fully integer-based operations to avoid floating-point errors
 * 
 * @param pin GPIO pin number
 * @param attenuation ADC attenuation
 * @param oversampling Number of samples to average (powers of 2 recommended)
 * @return Voltage in microvolts, or -1 on error
 */
int32_t analogReadMicrovolts(int pin, adc_atten_t attenuation, int oversampling) {
  int adc1_chan = get_adc1_channel(pin);
  if (adc1_chan == -1) {
    log_e("analogReadMicrovolts", "Invalid pin for ADC1: %d", pin);
    return -1;
  }

  // Configure ADC
  adc1_config_width(ADC_WIDTH_BIT_12); // Using 12-bit resolution
  esp_err_t err = adc1_config_channel_atten((adc1_channel_t)adc1_chan, attenuation);
  if (err != ESP_OK) {
    log_e("analogReadMicrovolts", "Error configuring attenuation for pin %d: %s", pin, esp_err_to_name(err));
    return -1;
  }

  // Initialize calibration data if needed
  if (!adc1_cal_initialized[adc1_chan] || adc1_cal_data[adc1_chan].current_atten != attenuation) {
    if (adc1_cal_data[adc1_chan].adc_chars != NULL) {
      free(adc1_cal_data[adc1_chan].adc_chars);
    }
    adc1_cal_data[adc1_chan].adc_chars = (esp_adc_cal_characteristics_t *)malloc(sizeof(esp_adc_cal_characteristics_t));
    if (adc1_cal_data[adc1_chan].adc_chars == NULL) {
      log_e("analogReadMicrovolts", "Memory allocation failed for calibration data");
      return -1;
    }
    esp_adc_cal_characterize(ADC_UNIT_1, attenuation, ADC_WIDTH_BIT_12, 3300, adc1_cal_data[adc1_chan].adc_chars);
    adc1_cal_data[adc1_chan].current_atten = attenuation;
    adc1_cal_initialized[adc1_chan] = true;
  }

  // Handle oversampling using integer-based Kahan summation
  uint32_t raw_adc = 0;
  
  // For microvolts, we use a much more precise approach with high oversampling
  if (oversampling > 1) {
    // Check if oversampling is a power of 2 (recommended)
    bool isPowerOf2 = (oversampling & (oversampling - 1)) == 0;
    int shift = 0;
    
    if (isPowerOf2) {
      int temp = oversampling;
      while (temp > 1) {
        temp >>= 1;
        shift++;
      }
    }
    
    // Use integer-based Kahan summation
    kahan_sum_t ksum;
    kahan_init(&ksum);
    
    for (int i = 0; i < oversampling; i++) {
      uint32_t sample = adc1_get_raw((adc1_channel_t)adc1_chan);
      kahan_add(&ksum, sample);
    }
    
    // For microvolts we want maximum precision in division
    if (isPowerOf2) {
      // Use rounding with bit shift
      raw_adc = (uint32_t)((ksum.sum + (1LL << (shift - 1))) >> shift);
    } else {
      raw_adc = (uint32_t)((ksum.sum + (oversampling >> 1)) / oversampling);
    }
  } else {
    raw_adc = adc1_get_raw((adc1_channel_t)adc1_chan);
  }

  // Bound check to avoid potential overflow issues
  if (raw_adc > 4095) {
    raw_adc = 4095;  // Cap at max 12-bit value
  }

  int32_t microvolts = 0;
  if (adc1_cal_initialized[adc1_chan] && adc1_cal_data[adc1_chan].adc_chars != NULL) {
    // Convert millivolts to microvolts using integer multiplication
    uint32_t millivolts = esp_adc_cal_raw_to_voltage(raw_adc, adc1_cal_data[adc1_chan].adc_chars);
    microvolts = static_cast<int32_t>(millivolts) * 1000;
  } else {
    // Calculate microvolts directly to avoid intermediate conversion errors
    // Fixed-point arithmetic: (raw_adc * 3300000) / 4095 with proper rounding
    microvolts = static_cast<int32_t>((static_cast<int64_t>(raw_adc) * 3300000LL + 2048LL) / 4095LL);
  }
  
  return microvolts;
}

/**
 * Extended precision analog read with high-accuracy oversampling
 * Uses 64-bit integer operations and optimized for maximum numerical stability
 * 
 * @param pin GPIO pin number
 * @param attenuation ADC attenuation
 * @param oversampling Number of samples (recommend powers of 2 ≥ 64 for best results)
 * @param results_array Optional pointer to store extended results [min, max, mean, stddev]
 * @return Voltage in microvolts, or -1 on error
 */
int32_t analogReadExtended(int pin, adc_atten_t attenuation, int oversampling, int32_t* results_array) {
  int adc1_chan = get_adc1_channel(pin);
  if (adc1_chan == -1) {
    log_e("analogReadExtended", "Invalid pin for ADC1: %d", pin);
    return -1;
  }

  // Configure ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  esp_err_t err = adc1_config_channel_atten((adc1_channel_t)adc1_chan, attenuation);
  if (err != ESP_OK) {
    log_e("analogReadExtended", "Error configuring attenuation for pin %d: %s", pin, esp_err_to_name(err));
    return -1;
  }

  // Ensure valid oversampling
  if (oversampling < 1) oversampling = 1;

  // Initialize calibration data if needed
  if (!adc1_cal_initialized[adc1_chan] || adc1_cal_data[adc1_chan].current_atten != attenuation) {
    if (adc1_cal_data[adc1_chan].adc_chars != NULL) {
      free(adc1_cal_data[adc1_chan].adc_chars);
    }
    adc1_cal_data[adc1_chan].adc_chars = (esp_adc_cal_characteristics_t *)malloc(sizeof(esp_adc_cal_characteristics_t));
    if (adc1_cal_data[adc1_chan].adc_chars == NULL) {
      log_e("analogReadExtended", "Memory allocation failed for calibration data");
      return -1;
    }
    esp_adc_cal_characterize(ADC_UNIT_1, attenuation, ADC_WIDTH_BIT_12, 3300, adc1_cal_data[adc1_chan].adc_chars);
    adc1_cal_data[adc1_chan].current_atten = attenuation;
    adc1_cal_initialized[adc1_chan] = true;
  }

  // Statistical variables - all integer-based
  uint32_t min_raw = UINT32_MAX;
  uint32_t max_raw = 0;
  kahan_sum_t ksum;
  kahan_init(&ksum);
  
  // For standard deviation calculation (all integer-based)
  kahan_sum_t sum_squares;
  kahan_init(&sum_squares);

  // Collect samples
  for (int i = 0; i < oversampling; i++) {
    uint32_t sample = adc1_get_raw((adc1_channel_t)adc1_chan);
    
    // Update min/max
    if (sample < min_raw) min_raw = sample;
    if (sample > max_raw) max_raw = sample;
    
    // Update sums using integer-based Kahan summation
    kahan_add(&ksum, sample);
    kahan_add(&sum_squares, static_cast<int64_t>(sample) * sample); // Square for stddev
  }

  // Calculate mean with proper rounding
  uint32_t mean_raw;
  if (oversampling == 1) {
    mean_raw = static_cast<uint32_t>(ksum.sum);
  } else {
    mean_raw = static_cast<uint32_t>((ksum.sum + (oversampling >> 1)) / oversampling);
  }

  // Calculate standard deviation using integer math
  uint32_t stddev_raw = 0;
  if (oversampling > 1) {
    // stddev = sqrt((sum_squares/n) - (mean*mean))
    int64_t mean_squared = static_cast<int64_t>(mean_raw) * mean_raw;
    int64_t variance = (sum_squares.sum / oversampling) - mean_squared;
    
    if (variance > 0) {
      // Integer square root approximation (Newton's method)
      int64_t x = variance;
      int64_t y = (x + 1) >> 1;
      while (y < x) {
        x = y;
        y = (x + variance / x) >> 1;
      }
      stddev_raw = static_cast<uint32_t>(x);
    }
  }

  // Convert raw values to voltages
  int32_t microvolts = 0;
  int32_t min_uv = 0, max_uv = 0, stddev_uv = 0;

  if (adc1_cal_initialized[adc1_chan] && adc1_cal_data[adc1_chan].adc_chars != NULL) {
    // Use calibration for better accuracy
    microvolts = static_cast<int32_t>(esp_adc_cal_raw_to_voltage(mean_raw, adc1_cal_data[adc1_chan].adc_chars)) * 1000;
    
    if (results_array != NULL) {
      min_uv = static_cast<int32_t>(esp_adc_cal_raw_to_voltage(min_raw, adc1_cal_data[adc1_chan].adc_chars)) * 1000;
      max_uv = static_cast<int32_t>(esp_adc_cal_raw_to_voltage(max_raw, adc1_cal_data[adc1_chan].adc_chars)) * 1000;
      stddev_uv = static_cast<int32_t>(esp_adc_cal_raw_to_voltage(stddev_raw, adc1_cal_data[adc1_chan].adc_chars)) * 1000;
    }
  } else {
    // Use fixed-point arithmetic for high precision
    microvolts = static_cast<int32_t>((static_cast<int64_t>(mean_raw) * 3300000LL + 2048LL) / 4095LL);
    
    if (results_array != NULL) {
      min_uv = static_cast<int32_t>((static_cast<int64_t>(min_raw) * 3300000LL + 2048LL) / 4095LL);
      max_uv = static_cast<int32_t>((static_cast<int64_t>(max_raw) * 3300000LL + 2048LL) / 4095LL);
      stddev_uv = static_cast<int32_t>((static_cast<int64_t>(stddev_raw) * 3300000LL + 2048LL) / 4095LL);
    }
  }
  
  // Store extended results if pointer provided
  if (results_array != NULL) {
    results_array[0] = min_uv;
    results_array[1] = max_uv;
    results_array[2] = microvolts;  // mean
    results_array[3] = stddev_uv;
  }
  
  return microvolts;
}
