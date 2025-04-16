#include "adc_utils.h"
#include "driver/gpio.h"        // For gpio_num_t
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdlib.h> // For malloc, free, qsort
#include <string.h> // For memset
#include <limits.h> // For INT32_MIN etc.

// For internal temperature sensor
#if CONFIG_IDF_TARGET_ESP32
#include "driver/temperature_sensor.h"
#endif

// --- Constants ---
static const char *TAG = "ADC_UTILS";
#define ADC_MAX_VALUE_F ((float)ADC_MAX_RAW)
#define UV_CONVERSION 1000LL // Use LL for 64-bit intermediate calcs

// --- Internal Structures ---

// Structure to store calibration data per channel
typedef struct {
    esp_adc_cal_characteristics_t* adc_chars;
    adc_atten_t current_atten;
    uint32_t vref; // Store actual reference voltage from calibration
    bool is_calibrated; // True if calibration characteristics are valid
    bool is_initialized; // True if channel has been configured at least once
} adc_calibration_t;

// Kahan summation state for high-precision summation
typedef struct {
    int64_t sum;
    int64_t correction;
} kahan_sum_t;

// ADC readings statistics
typedef struct {
    uint32_t min_raw;
    uint32_t max_raw;
    int64_t sum_squares; // Sum of squares for variance calculation
    kahan_sum_t sum_kahan; // Use Kahan sum for the primary sum
    int samples;
} adc_stats_t;

// EMA Filter state (fixed-point 24.8 format)
#define EMA_FIXED_POINT_BITS 8
#define EMA_SCALE_FACTOR (1 << EMA_FIXED_POINT_BITS)
#define EMA_ROUNDING_TERM (1 << (EMA_FIXED_POINT_BITS - 1))
typedef struct {
    int64_t ema_value_fixed;
    bool initialized;
} ema_state_t;


// --- Static Variables ---

// Calibration data for all ADC1 channels
static adc_calibration_t adc1_cal_data[ADC1_CHANNEL_MAX] = {0};
// Spinlock for thread safety when accessing/modifying calibration data or ADC config
static portMUX_TYPE adc_spinlock = portMUX_INITIALIZER_UNLOCKED;
// EMA filter state per ADC1 channel
static ema_state_t adc1_ema_state[ADC1_CHANNEL_MAX] = {0};
// Flag for internal temperature sensor initialization
static bool temp_sensor_initialized = false;

// --- Helper Functions ---

/**
 * @brief Maps a GPIO pin number to its corresponding ADC1 channel.
 *
 * @param pin GPIO pin number.
 * @return ADC1 channel enum, or ADC1_CHANNEL_MAX if the pin is not a valid ADC1 pin.
 * @note Verify this mapping against your specific ESP32 board/variant datasheet.
 * This is a common mapping for ESP32-WROOM/WROVER.
 */
static adc1_channel_t gpio_to_adc1_channel(int pin) {
    switch (pin) {
        case 36: return ADC1_CHANNEL_0; // SVP
        case 37: return ADC1_CHANNEL_1; // SVN
        case 38: return ADC1_CHANNEL_2;
        case 39: return ADC1_CHANNEL_3; // HALLN
        case 32: return ADC1_CHANNEL_4;
        case 33: return ADC1_CHANNEL_5;
        case 34: return ADC1_CHANNEL_6;
        case 35: return ADC1_CHANNEL_7;
        default: return ADC1_CHANNEL_MAX; // Invalid ADC1 pin
    }
}

static inline void kahan_init(kahan_sum_t* ksum) {
    if (ksum) {
        ksum->sum = 0;
        ksum->correction = 0;
    }
}

static inline void kahan_add(kahan_sum_t* ksum, int32_t value) {
    if (!ksum) return;
    // Use 64-bit integers to prevent overflow during intermediate calculations
    int64_t y = (int64_t)value - ksum->correction;
    int64_t t = ksum->sum + y;
    ksum->correction = (t - ksum->sum) - y;
    ksum->sum = t;
}

static inline uint8_t get_log2_power_of_2(uint32_t value) {
    if (value == 0 || (value & (value - 1)) != 0) return 0; // Not a power of 2 or zero
    uint8_t shift = 0;
    while (value > 1) {
        value >>= 1;
        shift++;
    }
    return shift;
}

/**
 * @brief Initialize or update ADC calibration data for a specific channel and attenuation.
 * Handles memory allocation/deallocation and thread safety.
 */
static esp_err_t init_adc_calibration(adc1_channel_t adc1_chan, adc_atten_t attenuation) {
    if (adc1_chan < 0 || adc1_chan >= ADC1_CHANNEL_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    bool needs_update = false;
    bool needs_alloc = false;

    portENTER_CRITICAL(&adc_spinlock);
    adc_calibration_t *cal = &adc1_cal_data[adc1_chan];

    // Check if we need to (re)calibrate or (re)allocate
    if (!cal->is_initialized || cal->current_atten != attenuation || !cal->adc_chars) {
        needs_update = true;
        if (!cal->adc_chars) {
             needs_alloc = true;
        } else {
            // Attenuation changed, free old calibration data if it existed
            free(cal->adc_chars);
            cal->adc_chars = NULL;
            needs_alloc = true;
        }
    }

    if (needs_update) {
        if (needs_alloc) {
            cal->adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
            if (cal->adc_chars == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for ADC calibration Ch %d", adc1_chan);
                // Mark as uninitialized but don't prevent fallback calculation
                cal->is_initialized = true; // It was attempted
                cal->is_calibrated = false;
                cal->current_atten = attenuation; // Store requested atten
                cal->vref = DEFAULT_VREF;
                portEXIT_CRITICAL(&adc_spinlock);
                return ESP_ERR_NO_MEM;
            }
        }

        // Characterize ADC
        cal->vref = DEFAULT_VREF; // Start with default
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
            ADC_UNIT_1, attenuation, ADC_WIDTH_BIT_12, DEFAULT_VREF, cal->adc_chars);

        cal->is_calibrated = false; // Assume calibration failed initially
        if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
            ESP_LOGI(TAG, "Ch %d: Using eFuse Vref: %d mV", adc1_chan, cal->adc_chars->vref);
            cal->vref = cal->adc_chars->vref;
            cal->is_calibrated = true;
        } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
            ESP_LOGI(TAG, "Ch %d: Using Two Point eFuse calibration", adc1_chan);
             cal->vref = cal->adc_chars->vref; // vref might still be estimated
             cal->is_calibrated = true;
        } else {
            ESP_LOGW(TAG, "Ch %d: ADC calibration not available or failed, using default Vref: %d mV", adc1_chan, DEFAULT_VREF);
            // Keep default vref, adc_chars might still contain approximated curve
            cal->vref = DEFAULT_VREF;
            // We can still use the approximated curve if `characterize` didn't return error
            if (cal->adc_chars->lin_coeffic == NULL) { // Check if characterize failed badly
                 ESP_LOGE(TAG, "Ch %d: ADC Characterization failed, cannot use calibration struct.", adc1_chan);
                 free(cal->adc_chars);
                 cal->adc_chars = NULL;
                 cal->is_calibrated = false;
            } else {
                 cal->is_calibrated = true; // Use approximated curve
                 ESP_LOGW(TAG, "Ch %d: Using approximated calibration curve.", adc1_chan);
            }
        }
        cal->current_atten = attenuation;
        cal->is_initialized = true;
    }
    portEXIT_CRITICAL(&adc_spinlock);
    return err; // Return ESP_OK mostly, errors handled via logs/state flags
}

/**
 * @brief Configures ADC pin and attenuation, initializes calibration if needed.
 */
static esp_err_t configure_adc_pin(int pin, adc_atten_t attenuation, adc1_channel_t *adc1_chan_out) {
    adc1_channel_t adc1_chan = gpio_to_adc1_channel(pin);
    if (adc1_chan == ADC1_CHANNEL_MAX) {
        ESP_LOGE(TAG, "Invalid GPIO pin %d for ADC1.", pin);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;
    portENTER_CRITICAL(&adc_spinlock); // Protect ADC configuration
    err = adc1_config_width(ADC_WIDTH_BIT_12);
    if (err != ESP_OK) {
        portEXIT_CRITICAL(&adc_spinlock);
        ESP_LOGE(TAG, "Failed to configure ADC width for pin %d: %s", pin, esp_err_to_name(err));
        return err;
    }
    err = adc1_config_channel_atten(adc1_chan, attenuation);
    if (err != ESP_OK) {
        portEXIT_CRITICAL(&adc_spinlock);
        ESP_LOGE(TAG, "Failed to configure attenuation for pin %d (Ch %d): %s", pin, adc1_chan, esp_err_to_name(err));
        return err;
    }
    portEXIT_CRITICAL(&adc_spinlock);

    // Initialize calibration data (handles its own locking)
    err = init_adc_calibration(adc1_chan, attenuation);
    // No Mem is logged inside init, other errors are warnings here.
    if (err != ESP_OK && err != ESP_ERR_NO_MEM) {
        ESP_LOGW(TAG, "Calibration issue for pin %d (Ch %d): %s. Using defaults/approximation.", pin, adc1_chan, esp_err_to_name(err));
        // Proceed even if calibration has issues, raw_to_microvolts has fallback
    }

    *adc1_chan_out = adc1_chan;
    return ESP_OK; // Configuration succeeded, even if calibration has warnings
}


/**
 * @brief Collects ADC readings and calculates basic statistics using Kahan summation.
 */
static esp_err_t collect_adc_readings(adc1_channel_t adc1_chan, int oversampling, adc_stats_t* stats) {
    if (stats == NULL || adc1_chan < 0 || adc1_chan >= ADC1_CHANNEL_MAX || oversampling <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(stats, 0, sizeof(adc_stats_t));
    stats->min_raw = UINT32_MAX;
    stats->samples = oversampling;
    kahan_init(&stats->sum_kahan);

    uint32_t sample_raw;
    int64_t sample_raw_64;

    for (int i = 0; i < oversampling; i++) {
        // Reading ADC should be relatively safe without spinlock IF only one task reads ONE channel.
        // However, configuration changes ARE protected by the spinlock in configure_adc_pin.
        // If multiple tasks might read the *same* channel concurrently, a lock here might be needed.
        sample_raw = adc1_get_raw(adc1_chan);

        // Basic bounds check (ADC errors can sometimes return > 4095)
        if (sample_raw > ADC_MAX_RAW) {
             sample_raw = ADC_MAX_RAW;
             ESP_LOGV(TAG, "Ch %d raw reading %lu out of bounds, capped to %d", adc1_chan, (unsigned long)sample_raw, ADC_MAX_RAW);
        }

        // Update min/max
        if (sample_raw < stats->min_raw) stats->min_raw = sample_raw;
        if (sample_raw > stats->max_raw) stats->max_raw = sample_raw;

        // Update sums using Kahan for mean and direct sum for variance
        kahan_add(&stats->sum_kahan, sample_raw);
        sample_raw_64 = (int64_t)sample_raw;
        stats->sum_squares += sample_raw_64 * sample_raw_64;

        // Small delay might help reduce noise in some scenarios, but adds overhead.
        // Consider adding if noise is an issue and sampling rate allows.
        // vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_OK;
}

/**
 * @brief Calculate mean from Kahan sum with proper rounding.
 */
static uint32_t calculate_mean_from_stats(const adc_stats_t* stats) {
    if (stats == NULL || stats->samples == 0) return 0;

    int64_t sum = stats->sum_kahan.sum;
    int samples = stats->samples;

    // Use efficient shift for powers of 2, otherwise use 64-bit division
    uint8_t shift = get_log2_power_of_2(samples);
    if (shift > 0) {
        // Power of 2 division with rounding: (sum + (divisor / 2)) / divisor
        return (uint32_t)((sum + (1LL << (shift - 1))) >> shift);
    } else {
        // Integer division with rounding: (sum + (divisor / 2)) / divisor
        return (uint32_t)((sum + (samples / 2)) / samples);
    }
}

/**
 * @brief Calculate standard deviation (raw value) from ADC statistics.
 */
static uint32_t calculate_stddev_raw(const adc_stats_t* stats, uint32_t mean_raw) {
    if (stats == NULL || stats->samples <= 1) return 0;

    int64_t n = stats->samples;
    // Calculate variance using 64-bit integers: Var = (sum_sq / n) - mean^2
    // To avoid floating point: Var = (sum_sq * n - (sum)^2) / n^2 --> complicates things
    // Stick to: Var = (sum_sq / n) - mean^2
    int64_t mean_raw_64 = (int64_t)mean_raw;
    int64_t variance = (stats->sum_squares / n) - (mean_raw_64 * mean_raw_64);

    if (variance <= 0) return 0; // Variance can be slightly negative due to integer math/precision

    // Integer square root approximation (Newton's method or Babylonian method)
    int64_t root = variance;
    int64_t last_root = 0;
    // Iterate until convergence or reasonable precision
    for(int i=0; i<100 && root != last_root; ++i) { // Limit iterations
        last_root = root;
        root = (root + variance / root) >> 1; // Equivalent to / 2
    }

    return (uint32_t)root;
}

/**
 * @brief Convert raw ADC value to microvolts using calibration or fallback approximation.
 */
static int32_t raw_to_microvolts(uint32_t raw_value, adc1_channel_t adc1_chan) {
    if (adc1_chan < 0 || adc1_chan >= ADC1_CHANNEL_MAX) {
        return 0; // Or INT32_MIN ? Return 0 for safety.
    }

    // Clip raw value just in case
    if (raw_value > ADC_MAX_RAW) raw_value = ADC_MAX_RAW;

    portENTER_CRITICAL(&adc_spinlock); // Access calibration data safely
    adc_calibration_t *cal = &adc1_cal_data[adc1_chan];
    bool calibrated = cal->is_calibrated && cal->adc_chars != NULL;
    uint32_t vref = cal->vref; // Get Vref determined during init
    esp_adc_cal_characteristics_t* chars_copy = NULL;

    // If calibrated, copy characteristics to use outside lock (esp_adc_cal functions might take time)
    if (calibrated) {
         // Create a temporary copy of the characteristics to use outside the critical section
         // Note: This assumes esp_adc_cal_raw_to_voltage doesn't rely on internal state accessed by characterize
         chars_copy = (esp_adc_cal_characteristics_t*)malloc(sizeof(esp_adc_cal_characteristics_t));
         if (chars_copy) {
            memcpy(chars_copy, cal->adc_chars, sizeof(esp_adc_cal_characteristics_t));
         } else {
             ESP_LOGE(TAG, "Ch %d: Failed to alloc temp copy for cal data, using fallback.", adc1_chan);
             calibrated = false; // Force fallback if copy fails
         }
    }
    portEXIT_CRITICAL(&adc_spinlock);

    int32_t microvolts;
    if (calibrated && chars_copy) {
        // Use ESP-IDF calibration function
        uint32_t millivolts = esp_adc_cal_raw_to_voltage(raw_value, chars_copy);
        free(chars_copy); // Free the temporary copy
        microvolts = (int32_t)millivolts * (UV_CONVERSION / 1000); // Convert mV to uV
    } else {
        // Fallback: Use high-precision fixed-point arithmetic with Vref (calibrated or default)
        // microvolts = (raw * vref * 1000) / ADC_MAX_RAW
        int64_t temp_uv = (int64_t)raw_value * vref * (UV_CONVERSION / 1000); // vref is mV, factor is 1000 uV/mV
        // Add half of the divisor for rounding before division
        temp_uv += (ADC_MAX_RAW / 2);
        microvolts = (int32_t)(temp_uv / ADC_MAX_RAW);
    }

    return microvolts;
}


// --- Public API Functions ---

int32_t analogReadMillivolts(int pin, adc_atten_t attenuation, int oversampling) {
    if (oversampling < 1) oversampling = 1;

    adc1_channel_t adc1_chan;
    if (configure_adc_pin(pin, attenuation, &adc1_chan) != ESP_OK) {
        return -1; // Consistent error return for voltage functions
    }

    adc_stats_t stats;
    if (collect_adc_readings(adc1_chan, oversampling, &stats) != ESP_OK) {
        return -1;
    }

    uint32_t mean_raw = calculate_mean_from_stats(&stats);
    int32_t microvolts = raw_to_microvolts(mean_raw, adc1_chan);

    if (microvolts == INT32_MIN) return -1; // Propagate error

    // Convert microvolts to millivolts with rounding
    int32_t millivolts = (microvolts + (int32_t)UV_CONVERSION / 2) / (int32_t)UV_CONVERSION;

    return millivolts;
}

int32_t analogReadMicrovolts(int pin, adc_atten_t attenuation, int oversampling) {
    if (oversampling < 1) oversampling = 1;
    if (oversampling < 4) {
        ESP_LOGD(TAG, "Microvolt reading recommend >= 4 samples (got %d)", oversampling);
    }

    adc1_channel_t adc1_chan;
    if (configure_adc_pin(pin, attenuation, &adc1_chan) != ESP_OK) {
        return INT32_MIN; // Use INT32_MIN as error marker for uV
    }

    adc_stats_t stats;
    if (collect_adc_readings(adc1_chan, oversampling, &stats) != ESP_OK) {
        return INT32_MIN;
    }

    uint32_t mean_raw = calculate_mean_from_stats(&stats);
    return raw_to_microvolts(mean_raw, adc1_chan);
}

int32_t analogReadExtended(int pin, adc_atten_t attenuation, int oversampling, int32_t* results_uv) {
    if (oversampling < 1) oversampling = 1;
     if (oversampling < 16) {
        ESP_LOGD(TAG, "Extended stats recommend >= 16 samples (got %d)", oversampling);
    }

    adc1_channel_t adc1_chan;
    if (configure_adc_pin(pin, attenuation, &adc1_chan) != ESP_OK) {
        if(results_uv) memset(results_uv, 0, 4 * sizeof(int32_t));
        return INT32_MIN;
    }

    adc_stats_t stats;
    if (collect_adc_readings(adc1_chan, oversampling, &stats) != ESP_OK) {
         if(results_uv) memset(results_uv, 0, 4 * sizeof(int32_t));
        return INT32_MIN;
    }

    uint32_t mean_raw = calculate_mean_from_stats(&stats);
    int32_t mean_uv = raw_to_microvolts(mean_raw, adc1_chan);

    if (results_uv != NULL) {
        uint32_t stddev_raw = calculate_stddev_raw(&stats, mean_raw);

        results_uv[0] = raw_to_microvolts(stats.min_raw, adc1_chan); // min_uv
        results_uv[1] = raw_to_microvolts(stats.max_raw, adc1_chan); // max_uv
        results_uv[2] = mean_uv;                                      // mean_uv

        // Note: Converting stddev_raw directly gives a value scaled like voltage,
        // not the strict standard deviation *of* the voltage readings.
        // For true voltage stddev, calculate from voltage samples or scale by (vref / max_raw).
        // This provides a measure of the noise/variation on the voltage scale.
        results_uv[3] = raw_to_microvolts(stddev_raw, adc1_chan);     // stddev_uv (scaled)
    }

    return mean_uv; // Return mean microvolts
}

// Comparison function for qsort
static int compare_uint16(const void *a, const void *b) {
    uint16_t arg1 = *(const uint16_t*)a;
    uint16_t arg2 = *(const uint16_t*)b;
    if (arg1 < arg2) return -1;
    if (arg1 > arg2) return 1;
    return 0;
}

int32_t analogReadMedian(int pin, adc_atten_t attenuation, int samples) {
    if (samples < 3) samples = 3;
    if (samples % 2 == 0) {
        samples++; // Ensure odd number for a true median
        ESP_LOGD(TAG, "Median samples adjusted to %d (odd number required)", samples);
    }

    adc1_channel_t adc1_chan;
    if (configure_adc_pin(pin, attenuation, &adc1_chan) != ESP_OK) {
        return INT32_MIN;
    }

    // Allocate buffer for samples
    uint16_t* readings = (uint16_t*)malloc(samples * sizeof(uint16_t));
    if (readings == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for median filter (%d samples)", samples);
        return INT32_MIN;
    }

    // Collect samples
    for (int i = 0; i < samples; i++) {
        readings[i] = adc1_get_raw(adc1_chan);
        if (readings[i] > ADC_MAX_RAW) {
            readings[i] = ADC_MAX_RAW;
        }
        // Optional small delay
        // ets_delay_us(10);
    }

    // Sort the readings - qsort is generally efficient
    qsort(readings, samples, sizeof(uint16_t), compare_uint16);

    // Get the median value (middle element)
    uint16_t median_raw = readings[samples / 2];
    free(readings);

    return raw_to_microvolts(median_raw, adc1_chan);
}


int32_t analogReadEMA(int pin, adc_atten_t attenuation, uint8_t alpha, bool reset) {
    adc1_channel_t adc1_chan;
    if (configure_adc_pin(pin, attenuation, &adc1_chan) != ESP_OK) {
        return INT32_MIN;
    }

    // Get current raw reading
    uint32_t current_raw = adc1_get_raw(adc1_chan);
    if (current_raw > ADC_MAX_RAW) current_raw = ADC_MAX_RAW;

    // Convert current reading to fixed point (24.8)
    int64_t current_raw_fixed = (int64_t)current_raw << EMA_FIXED_POINT_BITS;

    // Access/Update EMA state safely
    portENTER_CRITICAL(&adc_spinlock);
    ema_state_t *state = &adc1_ema_state[adc1_chan];

    // Reset or initialize EMA if needed
    if (reset || !state->initialized) {
        state->ema_value_fixed = current_raw_fixed;
        state->initialized = true;
        ESP_LOGD(TAG, "Ch %d EMA filter initialized/reset to %lld (raw %lu)", adc1_chan, state->ema_value_fixed, current_raw);
    } else {
        // Apply EMA filter: EMA_new = (alpha * current + (ScaleFactor - alpha) * EMA_old) / ScaleFactor
        // Using 64-bit integers to avoid overflow during calculation
        int64_t alpha64 = (int64_t)alpha;
        int64_t scale_minus_alpha = (int64_t)EMA_SCALE_FACTOR - alpha64;

        int64_t term1 = alpha64 * current_raw_fixed;
        int64_t term2 = scale_minus_alpha * state->ema_value_fixed;

        // Combine and scale down with rounding
        state->ema_value_fixed = (term1 + term2 + EMA_ROUNDING_TERM) >> EMA_FIXED_POINT_BITS;

        // state->ema_value_fixed = (alpha64 * current_raw_fixed + scale_minus_alpha * state->ema_value_fixed + EMA_ROUNDING_TERM) >> EMA_FIXED_POINT_BITS;
    }
    // Get the EMA value to convert outside the lock
    int64_t ema_fixed = state->ema_value_fixed;
    portEXIT_CRITICAL(&adc_spinlock);


    // Convert fixed-point EMA back to raw ADC value (integer part) with rounding
    uint32_t ema_raw = (uint32_t)((ema_fixed + EMA_ROUNDING_TERM) >> EMA_FIXED_POINT_BITS);

    // Clamp just in case of potential overshoot due to fixed point math? Should be unlikely with 64bit intermediates.
    if (ema_raw > ADC_MAX_RAW) ema_raw = ADC_MAX_RAW;


    return raw_to_microvolts(ema_raw, adc1_chan);
}

esp_err_t initInternalTemperatureSensor(void) {
 #if CONFIG_IDF_TARGET_ESP32 && CONFIG_ESP32_TEMP_SENSOR_ENABLED
    if (temp_sensor_initialized) {
        return ESP_OK; // Already initialized
    }
    ESP_LOGI(TAG, "Initializing internal temperature sensor");
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50); // Default dac_offset, clk_div
    esp_err_t err = temperature_sensor_install(&temp_sensor_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install temperature sensor: %s", esp_err_to_name(err));
        return err;
    }
     err = temperature_sensor_enable();
     if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable temperature sensor: %s", esp_err_to_name(err));
        temperature_sensor_uninstall(); // Clean up install if enable fails
        return err;
     }
     temp_sensor_initialized = true;
     return ESP_OK;
 #else
    ESP_LOGE(TAG, "Internal temperature sensor not supported/enabled on this target or configuration.");
    return ESP_ERR_NOT_SUPPORTED;
 #endif
}


int16_t readInternalTemperature(int samples) {
#if CONFIG_IDF_TARGET_ESP32 && CONFIG_ESP32_TEMP_SENSOR_ENABLED
    if (!temp_sensor_initialized) {
        ESP_LOGE(TAG, "Internal temperature sensor not initialized. Call initInternalTemperatureSensor() first.");
        return INT16_MIN;
    }
     if (samples < 1) samples = 1;

    float sum_temp = 0;
    float current_temp = 0;
    esp_err_t err;

    for (int i = 0; i < samples; i++) {
        err = temperature_sensor_get_celsius(&current_temp);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read temperature sensor: %s", esp_err_to_name(err));
            // Consider disabling sensor? For now, just return error.
            // temperature_sensor_disable();
            // temp_sensor_initialized = false;
            return INT16_MIN;
        }
        sum_temp += current_temp;
        // Small delay between readings might be good practice
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    float avg_temp = sum_temp / samples;

    // Return temperature in Celsius * 100
    return (int16_t)(avg_temp * 100.0f);

#else
    ESP_LOGE(TAG, "Internal temperature sensor not supported/enabled on this target or configuration.");
    return INT16_MIN; // Indicate error or not supported
#endif
}


uint32_t getADCVref(void) {
    uint32_t vref = DEFAULT_VREF;
    bool found = false;

    portENTER_CRITICAL(&adc_spinlock);
    // Find the Vref from the first initialized channel
    for (int i = 0; i < ADC1_CHANNEL_MAX; i++) {
        if (adc1_cal_data[i].is_initialized) {
            vref = adc1_cal_data[i].vref; // Use the stored Vref
            found = true;
            break;
        }
    }
    portEXIT_CRITICAL(&adc_spinlock);

    if (!found) {
         ESP_LOGD(TAG, "Vref requested but no channel initialized yet, returning default %d mV", DEFAULT_VREF);
    }

    return vref;
}

void analogCleanup(void) {
    portENTER_CRITICAL(&adc_spinlock);
    for (int i = 0; i < ADC1_CHANNEL_MAX; i++) {
        if (adc1_cal_data[i].adc_chars != NULL) {
            free(adc1_cal_data[i].adc_chars);
            adc1_cal_data[i].adc_chars = NULL;
        }
        // Reset state completely
        memset(&adc1_cal_data[i], 0, sizeof(adc_calibration_t));
        memset(&adc1_ema_state[i], 0, sizeof(ema_state_t));
    }
    portEXIT_CRITICAL(&adc_spinlock);

    // Also disable and uninstall temperature sensor if it was used
#if CONFIG_IDF_TARGET_ESP32 && CONFIG_ESP32_TEMP_SENSOR_ENABLED
    if (temp_sensor_initialized) {
        ESP_LOGI(TAG, "Disabling internal temperature sensor");
        temperature_sensor_disable();
        temperature_sensor_uninstall();
        temp_sensor_initialized = false;
    }
#endif
    ESP_LOGI(TAG, "ADC utilities cleaned up.");
}
