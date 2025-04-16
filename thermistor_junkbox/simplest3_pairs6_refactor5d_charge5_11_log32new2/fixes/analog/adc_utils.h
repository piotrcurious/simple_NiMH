#ifndef ADC_UTILS_H
#define ADC_UTILS_H

#include "esp_adc_cal.h" // For adc_atten_t
#include <stdint.h>
#include <stdbool.h>

// --- Configuration ---
// Default reference voltage in mV if calibration fails or is unavailable
#define DEFAULT_VREF 3300
// Max raw ADC value (for 12-bit resolution)
#define ADC_MAX_RAW 4095

// --- Public Function Prototypes ---

/**
 * @brief Reads the ADC pin voltage in millivolts with oversampling.
 *
 * Uses averaging with Kahan summation for improved numerical stability.
 * Attempts to use eFuse calibration if available.
 *
 * @param pin GPIO pin number connected to ADC1.
 * @param attenuation ADC attenuation setting (e.g., ADC_ATTEN_DB_11).
 * @param oversampling Number of samples to average (powers of 2 recommended for efficiency). Minimum 1.
 * @return Voltage in millivolts, or -1 on error.
 */
int32_t analogReadMillivolts(int pin, adc_atten_t attenuation, int oversampling);

/**
 * @brief Reads the ADC pin voltage in microvolts with oversampling.
 *
 * Uses averaging with Kahan summation and high-precision integer math.
 * Attempts to use eFuse calibration if available.
 *
 * @param pin GPIO pin number connected to ADC1.
 * @param attenuation ADC attenuation setting (e.g., ADC_ATTEN_DB_11).
 * @param oversampling Number of samples to average (powers of 2 recommended). Minimum 4 suggested.
 * @return Voltage in microvolts, or INT32_MIN on error.
 */
int32_t analogReadMicrovolts(int pin, adc_atten_t attenuation, int oversampling);

/**
 * @brief Extended precision analog read with statistics.
 *
 * Provides min, max, mean, and standard deviation of readings over multiple samples.
 * Uses Kahan summation and 64-bit integers for high numerical stability.
 *
 * @param pin GPIO pin number connected to ADC1.
 * @param attenuation ADC attenuation setting.
 * @param oversampling Number of samples (recommend powers of 2 >= 16). Minimum 16 suggested.
 * @param results_uv Optional pointer to an int32_t array of size 4.
 * If provided, it will be filled with:
 * [0]: Minimum voltage (uV)
 * [1]: Maximum voltage (uV)
 * [2]: Mean voltage (uV)
 * [3]: Standard deviation (scaled like voltage, in uV)
 * @return Mean voltage in microvolts, or INT32_MIN on error.
 */
int32_t analogReadExtended(int pin, adc_atten_t attenuation, int oversampling, int32_t* results_uv);

/**
 * @brief Reads the ADC pin voltage using median filtering.
 *
 * Effective for removing impulse noise or outliers.
 *
 * @param pin GPIO pin number connected to ADC1.
 * @param attenuation ADC attenuation setting.
 * @param samples Number of samples for median calculation (odd number recommended, min 3).
 * @return Median voltage in microvolts, or INT32_MIN on error.
 */
int32_t analogReadMedian(int pin, adc_atten_t attenuation, int samples);

/**
 * @brief Reads the ADC pin voltage using an Exponential Moving Average (EMA) filter.
 *
 * Good for smoothing noisy signals that change relatively slowly. Maintains state per channel.
 *
 * @param pin GPIO pin number connected to ADC1.
 * @param attenuation ADC attenuation setting.
 * @param alpha Smoothing factor (0-255). 0 = no filtering (current reading), 255 = max smoothing (very slow update). A common starting value is 10-50.
 * @param reset If true, resets the filter's internal state for this channel to the current reading.
 * @return Filtered voltage in microvolts, or INT32_MIN on error.
 */
int32_t analogReadEMA(int pin, adc_atten_t attenuation, uint8_t alpha, bool reset);

/**
 * @brief Reads the ESP32's internal temperature sensor.
 *
 * NOTE: Requires CONFIG_ESP32_TEMP_SENSOR_ENABLED=y in sdkconfig.
 * Only available on ESP32, not S2/S3/C3 etc. using this specific API.
 * Needs initial call to setup.
 *
 * @param samples Number of internal readings to average (e.g., 4-8). Minimum 1.
 * @return Temperature in degrees Celsius * 100 (e.g., 2530 means 25.30°C), or INT16_MIN on error or if not supported.
 */
int16_t readInternalTemperature(int samples);

/**
 * @brief Initializes the internal temperature sensor system.
 * Must be called once before the first call to readInternalTemperature().
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t initInternalTemperatureSensor(void);

/**
 * @brief Gets the ADC reference voltage (VREF) estimated during calibration.
 *
 * Returns the VREF used for the first successfully calibrated channel,
 * or the default VREF if no calibration has been performed or succeeded.
 *
 * @return Reference voltage in millivolts.
 */
uint32_t getADCVref(void);

/**
 * @brief Cleans up allocated ADC calibration resources.
 *
 * Call this if you no longer need ADC readings to free memory.
 */
void analogCleanup(void);

#endif // ADC_UTILS_H
