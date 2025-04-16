#include "analog.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Constants for ADC operations
#define ADC_MAX_VALUE 4095  // Maximum 12-bit ADC value
#define DEFAULT_VREF 3300   // Default reference voltage in mV if calibration fails
#define UV_CONVERSION 1000  // Conversion factor from mV to μV

// Structure to store calibration data with proper memory management
typedef struct {
    esp_adc_cal_characteristics_t* adc_chars;
    adc_atten_t current_atten;
    uint32_t vref;  // Store actual reference voltage from calibration
    bool is_initialized;
} adc_calibration_t;

// Allocate calibration data for all ADC1 channels
static adc_calibration_t adc1_cal_data[ADC1_CHANNEL_COUNT] = {0};
static portMUX_TYPE adc_spinlock = portMUX_INITIALIZER_UNLOCKED;  // Thread safety for multi-core

// Structure to hold Kahan summation state for high-precision summation
typedef struct {
    int64_t sum;        // Current sum
    int64_t correction; // Running compensation term
} kahan_sum_t;

// Structure for storing ADC readings statistics 
typedef struct {
    uint32_t min_raw;   // Minimum raw reading
    uint32_t max_raw;   // Maximum raw reading
    int64_t sum;        // Sum of readings
    int64_t sum_squares; // Sum of squares for variance calculation
    int samples;        // Number of samples collected
} adc_stats_t;

/**
 * Initialize a Kahan summation structure
 * @param ksum Pointer to kahan_sum_t structure
 */
static inline void kahan_init(kahan_sum_t* ksum) {
    if (ksum != NULL) {
        ksum->sum = 0;
        ksum->correction = 0;
    }
}

/**
 * Add a value to Kahan sum using integer-only operations
 * @param ksum Pointer to kahan_sum_t structure
 * @param value Value to add
 */
static inline void kahan_add(kahan_sum_t* ksum, int32_t value) {
    if (ksum == NULL) return;
    
    int64_t y = value - ksum->correction;
    int64_t t = ksum->sum + y;
    ksum->correction = (t - ksum->sum) - y;
    ksum->sum = t;
}

/**
 * Get the log2 of a power-of-2 number for efficient division by shifting
 * @param value Power of 2 value
 * @return log2(value) or 0 if not a power of 2
 */
static inline uint8_t get_log2(uint32_t value) {
    if (value == 0 || (value & (value - 1))) return 0; // Not a power of 2
    
    uint8_t shift = 0;
    while (value > 1) {
        value >>= 1;
        shift++;
    }
    return shift;
}

/**
 * Initialize or update ADC calibration data
 * @param adc1_chan ADC1 channel number
 * @param attenuation ADC attenuation level
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t init_adc_calibration(int adc1_chan, adc_atten_t attenuation) {
    if (adc1_chan < 0 || adc1_chan >= ADC1_CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if we need to initialize or update calibration
    if (!adc1_cal_data[adc1_chan].is_initialized || 
        adc1_cal_data[adc1_chan].current_atten != attenuation) {
        
        portENTER_CRITICAL(&adc_spinlock);
        
        // Free existing calibration if it exists
        if (adc1_cal_data[adc1_chan].adc_chars != NULL) {
            free(adc1_cal_data[adc1_chan].adc_chars);
            adc1_cal_data[adc1_chan].adc_chars = NULL;
        }
        
        // Allocate new calibration data
        adc1_cal_data[adc1_chan].adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
        if (adc1_cal_data[adc1_chan].adc_chars == NULL) {
            portEXIT_CRITICAL(&adc_spinlock);
            return ESP_ERR_NO_MEM;
        }
        
        // Get actual Vref from eFuse if available
        uint32_t vref = DEFAULT_VREF;
        esp_err_t err = ESP_OK;
        
        #ifdef CONFIG_IDF_TARGET_ESP32
        // Try to get eFuse Vref calibration
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
            ADC_UNIT_1, attenuation, ADC_WIDTH_BIT_12, DEFAULT_VREF, 
            adc1_cal_data[adc1_chan].adc_chars);
            
        if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
            vref = adc1_cal_data[adc1_chan].adc_chars->vref;
        }
        #else
        // For ESP32-S2/S3/C3, use built-in calibration
        err = esp_adc_cal_characterize(
            ADC_UNIT_1, attenuation, ADC_WIDTH_BIT_12, DEFAULT_VREF, 
            adc1_cal_data[adc1_chan].adc_chars);
        #endif
        
        adc1_cal_data[adc1_chan].current_atten = attenuation;
        adc1_cal_data[adc1_chan].vref = vref;
        adc1_cal_data[adc1_chan].is_initialized = true;
        
        portEXIT_CRITICAL(&adc_spinlock);
        return err;
    }
    
    return ESP_OK;
}

/**
 * Configure ADC pin and retrieve ADC1 channel number
 * @param pin GPIO pin number
 * @param attenuation ADC attenuation
 * @return ADC1 channel number or -1 on error
 */
static int configure_adc_pin(int pin, adc_atten_t attenuation) {
    // Get ADC1 channel from pin
    int adc1_chan = get_adc1_channel(pin);
    if (adc1_chan == -1) {
        log_e("Invalid pin for ADC1: %d", pin);
        return -1;
    }

    // Configure ADC width and attenuation
    adc1_config_width(ADC_WIDTH_BIT_12);
    esp_err_t err = adc1_config_channel_atten((adc1_channel_t)adc1_chan, attenuation);
    if (err != ESP_OK) {
        log_e("Error configuring attenuation for pin %d: %s", pin, esp_err_to_name(err));
        return -1;
    }

    // Initialize calibration data
    err = init_adc_calibration(adc1_chan, attenuation);
    if (err != ESP_OK && err != ESP_ERR_NO_MEM) {
        log_w("Calibration warning for pin %d: %s", pin, esp_err_to_name(err));
        // Continue with default values even if calibration fails
    }

    return adc1_chan;
}

/**
 * Collect ADC readings with statistics
 * @param adc1_chan ADC1 channel number
 * @param oversampling Number of samples to collect
 * @param stats Pointer to statistics structure
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t collect_adc_readings(int adc1_chan, int oversampling, adc_stats_t* stats) {
    if (stats == NULL || adc1_chan < 0 || adc1_chan >= ADC1_CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize statistics structure
    stats->min_raw = UINT32_MAX;
    stats->max_raw = 0;
    stats->sum = 0;
    stats->sum_squares = 0;
    stats->samples = oversampling;
    
    // Use Kahan summation for numerical stability
    kahan_sum_t ksum_values, ksum_squares;
    kahan_init(&ksum_values);
    kahan_init(&ksum_squares);
    
    // Collect samples
    for (int i = 0; i < oversampling; i++) {
        // Get raw ADC reading
        uint32_t sample = adc1_get_raw((adc1_channel_t)adc1_chan);
        
        // Bound check (protect against ADC errors)
        if (sample > ADC_MAX_VALUE) {
            sample = ADC_MAX_VALUE;
        }
        
        // Update min/max
        if (sample < stats->min_raw) stats->min_raw = sample;
        if (sample > stats->max_raw) stats->max_raw = sample;
        
        // Update sums with Kahan summation
        kahan_add(&ksum_values, sample);
        kahan_add(&ksum_squares, (int64_t)sample * sample);
    }
    
    // Store sum values
    stats->sum = ksum_values.sum;
    stats->sum_squares = ksum_squares.sum;
    
    return ESP_OK;
}

/**
 * Calculate mean from ADC statistics
 * @param stats Pointer to statistics structure
 * @return Mean ADC value
 */
static uint32_t calculate_mean(const adc_stats_t* stats) {
    if (stats == NULL || stats->samples == 0) return 0;
    
    // Check if power of 2 for efficient division
    uint8_t shift = get_log2(stats->samples);
    
    if (shift > 0) {
        // Power of 2 division with rounding
        return (uint32_t)((stats->sum + (1LL << (shift - 1))) >> shift);
    } else {
        // Integer division with rounding
        return (uint32_t)((stats->sum + (stats->samples >> 1)) / stats->samples);
    }
}

/**
 * Calculate standard deviation from ADC statistics
 * @param stats Pointer to statistics structure
 * @param mean Mean value
 * @return Standard deviation or 0 if not applicable
 */
static uint32_t calculate_stddev(const adc_stats_t* stats, uint32_t mean) {
    if (stats == NULL || stats->samples <= 1) return 0;
    
    // Calculate variance: (sum_squares/n) - (mean*mean)
    int64_t mean_squared = (int64_t)mean * mean;
    int64_t variance = (stats->sum_squares / stats->samples) - mean_squared;
    
    if (variance <= 0) return 0;
    
    // Integer square root approximation (Newton's method)
    int64_t x = variance;
    int64_t y = (x + 1) >> 1;
    while (y < x) {
        x = y;
        y = (x + variance / x) >> 1;
    }
    
    return (uint32_t)x;
}

/**
 * Convert raw ADC value to microvolts using calibration or approximation
 * @param raw_value Raw ADC value
 * @param adc1_chan ADC1 channel
 * @return Voltage in microvolts
 */
static int32_t raw_to_microvolts(uint32_t raw_value, int adc1_chan) {
    if (adc1_chan < 0 || adc1_chan >= ADC1_CHANNEL_COUNT) {
        return 0;
    }
    
    // Check if calibration data is available
    if (adc1_cal_data[adc1_chan].is_initialized && adc1_cal_data[adc1_chan].adc_chars != NULL) {
        // Use calibration data to convert raw value to voltage
        uint32_t millivolts = esp_adc_cal_raw_to_voltage(raw_value, adc1_cal_data[adc1_chan].adc_chars);
        return (int32_t)millivolts * UV_CONVERSION;
    } else {
        // Use high-precision fixed-point arithmetic with actual Vref or default
        uint32_t vref = DEFAULT_VREF;
        if (adc1_cal_data[adc1_chan].is_initialized) {
            vref = adc1_cal_data[adc1_chan].vref;
        }
        
        // (raw * vref * 1000) / ADC_MAX_VALUE with proper rounding
        return (int32_t)(((int64_t)raw_value * vref * UV_CONVERSION + (ADC_MAX_VALUE >> 1)) / ADC_MAX_VALUE);
    }
}

/**
 * Read analog value in millivolts with improved numerical stability
 * @param pin GPIO pin number
 * @param attenuation ADC attenuation
 * @param oversampling Number of samples to average (use powers of 2 for efficient division)
 * @return Voltage in millivolts, or -1 on error
 */
int analogReadMillivolts(int pin, adc_atten_t attenuation, int oversampling) {
    // Ensure valid oversampling value
    if (oversampling < 1) oversampling = 1;
    
    // Configure ADC pin and get channel
    int adc1_chan = configure_adc_pin(pin, attenuation);
    if (adc1_chan == -1) return -1;
    
    // Collect ADC readings
    adc_stats_t stats;
    esp_err_t err = collect_adc_readings(adc1_chan, oversampling, &stats);
    if (err != ESP_OK) return -1;
    
    // Calculate mean value
    uint32_t mean_raw = calculate_mean(&stats);
    
    // Convert to microvolts and then to millivolts
    int32_t microvolts = raw_to_microvolts(mean_raw, adc1_chan);
    return microvolts / UV_CONVERSION;
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
    // Ensure valid oversampling value (use at least 4 samples for microvolts)
    if (oversampling < 1) oversampling = 4;
    
    // Configure ADC pin and get channel
    int adc1_chan = configure_adc_pin(pin, attenuation);
    if (adc1_chan == -1) return -1;
    
    // Collect ADC readings
    adc_stats_t stats;
    esp_err_t err = collect_adc_readings(adc1_chan, oversampling, &stats);
    if (err != ESP_OK) return -1;
    
    // Calculate mean value
    uint32_t mean_raw = calculate_mean(&stats);
    
    // Convert to microvolts with high precision
    return raw_to_microvolts(mean_raw, adc1_chan);
}

/**
 * Extended precision analog read with high-accuracy oversampling and statistical metrics
 * Uses 64-bit integer operations and optimized for maximum numerical stability
 * 
 * @param pin GPIO pin number
 * @param attenuation ADC attenuation
 * @param oversampling Number of samples (recommend powers of 2 ≥ 64 for best results)
 * @param results_array Optional pointer to store extended results [min, max, mean, stddev]
 * @return Voltage in microvolts, or -1 on error
 */
int32_t analogReadExtended(int pin, adc_atten_t attenuation, int oversampling, int32_t* results_array) {
    // Ensure high oversampling for extended precision (minimum 16 samples)
    if (oversampling < 16) oversampling = 16;
    
    // Configure ADC pin and get channel
    int adc1_chan = configure_adc_pin(pin, attenuation);
    if (adc1_chan == -1) return -1;
    
    // Collect ADC readings with full statistics
    adc_stats_t stats;
    esp_err_t err = collect_adc_readings(adc1_chan, oversampling, &stats);
    if (err != ESP_OK) return -1;
    
    // Calculate mean value
    uint32_t mean_raw = calculate_mean(&stats);
    
    // Calculate standard deviation if needed
    uint32_t stddev_raw = 0;
    if (results_array != NULL) {
        stddev_raw = calculate_stddev(&stats, mean_raw);
    }
    
    // Convert raw values to voltages in microvolts
    int32_t mean_uv = raw_to_microvolts(mean_raw, adc1_chan);
    
    // Fill results array if provided
    if (results_array != NULL) {
        results_array[0] = raw_to_microvolts(stats.min_raw, adc1_chan);  // min
        results_array[1] = raw_to_microvolts(stats.max_raw, adc1_chan);  // max
        results_array[2] = mean_uv;                                      // mean
        results_array[3] = raw_to_microvolts(stddev_raw, adc1_chan);     // stddev
    }
    
    return mean_uv;
}

/**
 * Clean up ADC calibration resources when no longer needed
 * Call this when shutting down or when ADC will not be used for a long time
 */
void analogCleanup(void) {
    portENTER_CRITICAL(&adc_spinlock);
    
    for (int i = 0; i < ADC1_CHANNEL_COUNT; i++) {
        if (adc1_cal_data[i].adc_chars != NULL) {
            free(adc1_cal_data[i].adc_chars);
            adc1_cal_data[i].adc_chars = NULL;
        }
        adc1_cal_data[i].is_initialized = false;
    }
    
    portEXIT_CRITICAL(&adc_spinlock);
}

/**
 * Read analog value with noise reduction using median filtering
 * Effective for removing outliers and impulse noise
 * 
 * @param pin GPIO pin number
 * @param attenuation ADC attenuation
 * @param samples Number of samples for median calculation (odd number recommended)
 * @return Voltage in microvolts, or -1 on error
 */
int32_t analogReadMedian(int pin, adc_atten_t attenuation, int samples) {
    // Ensure valid samples count (use odd number for true median)
    if (samples < 3) samples = 3;
    if (samples % 2 == 0) samples++; // Make odd for true median
    
    // Configure ADC pin and get channel
    int adc1_chan = configure_adc_pin(pin, attenuation);
    if (adc1_chan == -1) return -1;
    
    // Allocate buffer for samples
    uint16_t* readings = (uint16_t*)malloc(samples * sizeof(uint16_t));
    if (readings == NULL) {
        log_e("Failed to allocate memory for median filtering");
        return -1;
    }
    
    // Collect samples
    for (int i = 0; i < samples; i++) {
        readings[i] = adc1_get_raw((adc1_channel_t)adc1_chan);
        
        // Bound check
        if (readings[i] > ADC_MAX_VALUE) {
            readings[i] = ADC_MAX_VALUE;
        }
    }
    
    // Simple insertion sort (efficient for small arrays)
    for (int i = 1; i < samples; i++) {
        uint16_t key = readings[i];
        int j = i - 1;
        
        while (j >= 0 && readings[j] > key) {
            readings[j + 1] = readings[j];
            j--;
        }
        readings[j + 1] = key;
    }
    
    // Get median value
    uint16_t median = readings[samples / 2];
    free(readings);
    
    // Convert to microvolts
    return raw_to_microvolts(median, adc1_chan);
}

/**
 * Read analog value with exponential moving average filtering
 * Good for tracking slowly changing signals with noise reduction
 * 
 * @param pin GPIO pin number
 * @param attenuation ADC attenuation
 * @param alpha Smoothing factor (0-255, where 0=no filtering, 255=max filtering)
 * @param reset Force reset of EMA filter state
 * @return Voltage in microvolts, or -1 on error
 */
int32_t analogReadEMA(int pin, adc_atten_t attenuation, uint8_t alpha, bool reset) {
    // Static EMA state (one per ADC channel)
    static int64_t ema_values[ADC1_CHANNEL_COUNT] = {0};
    static bool ema_initialized[ADC1_CHANNEL_COUNT] = {false};
    
    // Configure ADC pin and get channel
    int adc1_chan = configure_adc_pin(pin, attenuation);
    if (adc1_chan == -1) return -1;
    
    // Get current raw reading
    uint32_t current_raw = adc1_get_raw((adc1_channel_t)adc1_chan);
    if (current_raw > ADC_MAX_VALUE) current_raw = ADC_MAX_VALUE;
    
    // Reset or initialize EMA if needed
    if (reset || !ema_initialized[adc1_chan]) {
        ema_values[adc1_chan] = ((int64_t)current_raw << 8); // Fixed-point representation (8-bit fraction)
        ema_initialized[adc1_chan] = true;
    }
    
    // Apply EMA filter: EMA = (alpha * current + (256-alpha) * EMA) / 256
    ema_values[adc1_chan] = ((int64_t)alpha * current_raw + (256 - alpha) * (ema_values[adc1_chan] >> 8)) << 8;
    ema_values[adc1_chan] >>= 8;
    
    // Convert fixed-point EMA to raw ADC value
    uint32_t ema_raw = (uint32_t)(ema_values[adc1_chan] >> 8);
    
    // Convert to microvolts
    return raw_to_microvolts(ema_raw, adc1_chan);
}

/**
 * Get the internal temperature sensor reading in Celsius
 * Uses the built-in temperature sensor available on ESP32
 * 
 * @param samples Number of samples to average
 * @return Temperature in Celsius multiplied by 10 (e.g., 250 = 25.0°C), or INT_MIN on error
 */
int16_t readInternalTemperature(int samples) {
    #if CONFIG_IDF_TARGET_ESP32
    // Configure temperature sensor (ADC1 channel 6 on ESP32)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);
    
    // Collect readings
    adc_stats_t stats;
    if (collect_adc_readings(6, samples, &stats) != ESP_OK) {
        return INT16_MIN;
    }
    
    // Calculate mean
    uint32_t raw_temp = calculate_mean(&stats);
    
    // Convert to temperature
    // T = (raw - 1172) / 10  (nominal)
    // Adjust based on ESP32 temperature sensor characteristics
    return (int16_t)(((int32_t)raw_temp - 1172) * 10 / 10);
    #else
    // Not supported on other ESP32 variants
    return INT16_MIN;
    #endif
}

/**
 * Get the reference voltage (VREF) from calibration
 * 
 * @return Reference voltage in millivolts or 0 if unavailable
 */
uint32_t getADCVref(void) {
    #ifdef CONFIG_IDF_TARGET_ESP32
    // Check if we have a calibrated Vref
    uint32_t vref = 0;
    esp_err_t err = esp_adc_cal_get_voltage(
        ADC1_CHANNEL_6, // Use a standard channel
        &adc1_cal_data[0].adc_chars,
        &vref
    );
    
    if (err == ESP_OK) {
        return vref;
    } else {
        // Try to get from eFuse
        esp_adc_cal_characteristics_t chars;
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
            ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, DEFAULT_VREF, &chars);
            
        if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
            return chars.vref;
        }
    }
    #endif
    
    return DEFAULT_VREF;  // Default if no calibration available
}
