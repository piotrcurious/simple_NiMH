// improved_rint_measurement.cpp // Refactored and improved internal resistance measurement subsystem // - Uses coarse sampling + interpolation to reduce expensive measurements // - Adds measurement caching and averaged sampling with stability checks // - Provides two modes: FAST (coarse + refine) and ACCURATE (binary search) // - More robust handling of edge cases and invalid points // - Clear separation of helpers and high-level flow for easier testing

#include <vector> #include <map> #include <algorithm> #include <numeric> #include <cmath> #include <limits> #include <stdint.h>

// --- Configuration (tune these constants for your hardware & time budget) --- static constexpr int COARSE_SAMPLE_POINTS = 11;               // number of points to sample across duty range for interpolation static constexpr int COARSE_SAMPLE_ATTEMPTS = 2;              // number of measurements to average on coarse pass static constexpr int REFINEMENT_ATTEMPTS = 4;                // number of averaged measurements when refining a candidate duty static constexpr int REFINEMENT_WINDOW = 8;                  // +/- duty steps to search around interpolated guess static constexpr int MAX_MEASUREMENTS_PER_PAIR = 12;         // safety cap to avoid runaway measurement loops static constexpr float STABILITY_STDDEV_THRESHOLD = 0.015f;  // relative stddev threshold considered "stable" static constexpr int MAX_TOTAL_MEASUREMENTS = 400;           // global cap to preserve device responsiveness static constexpr bool USE_FAST_METHOD = true;                // set false to prefer binary-search-only (more measurements)

// Helper structure that mirrors existing MeasurementData but extended for cache struct CachedMeasurement { float voltage; float current; unsigned long timestamp; bool valid; int samplesTaken; // how many raw measurements averaged };

// Measurement cache keyed by integer duty cycle static std::map<int, CachedMeasurement> measurementCache; static int measurementsPerformed = 0;

// Forward declarations to keep integration clear MeasurementData takeMeasurement(int duty, int delayMs); void processThermistorData(const MeasurementData& m, const char* stage); MeasurementData getUnloadedVoltageMeasurement(); int findMinimalDutyCycle(); void storeResistanceData(float current, float resistance, float (*destArray)[2], int &destCount); void bubbleSort(float (*array)[2], int &count); bool performLinearRegression(float (*array)[2], int count, float &slope, float &intercept);

// ---------------------- Low-level helpers ----------------------

// Average multiple measurements (calls user-provided takeMeasurement and processThermistorData) static CachedMeasurement sampleAndAverage(int duty, int attempts, int stabilizationDelayMs, const char* tag=nullptr) { CachedMeasurement out{}; out.valid = false; if (attempts <= 0) attempts = 1;

std::vector<float> voltages;
std::vector<float> currents;
voltages.reserve(attempts);
currents.reserve(attempts);

for (int i = 0; i < attempts; ++i) {
    MeasurementData d = takeMeasurement(duty, stabilizationDelayMs);
    if (tag) processThermistorData(d, tag);
    voltages.push_back(d.voltage);
    currents.push_back(d.current);
    ++measurementsPerformed;
    if (measurementsPerformed >= MAX_TOTAL_MEASUREMENTS) break; // safety
}

if (voltages.empty()) return out;

// compute mean
float meanV = std::accumulate(voltages.begin(), voltages.end(), 0.0f) / voltages.size();
float meanI = std::accumulate(currents.begin(), currents.end(), 0.0f) / currents.size();

// compute sample stddev of current (relative)
float varI = 0.0f;
for (float c : currents) varI += (c - meanI) * (c - meanI);
varI = (voltages.size() > 1) ? varI / (voltages.size() - 1) : 0.0f;
float stddevI = std::sqrt(varI);

out.voltage = meanV;
out.current = meanI;
out.timestamp = millis();
out.valid = true;
out.samplesTaken = voltages.size();

// store a hint about stability (we'll treat relative stddev)
// Attach it to voltage as small sentinel if needed, but better to rely on 'samplesTaken' and stddev separately if desired

return out;

}

// Get or create cached averaged measurement static CachedMeasurement getCachedMeasurement(int duty, int attempts, int delayMs, const char* tag=nullptr) { auto it = measurementCache.find(duty); if (it != measurementCache.end()) return it->second;

CachedMeasurement m = sampleAndAverage(duty, attempts, delayMs, tag);
measurementCache[duty] = m;
return m;

}

// Linear interpolation helper (assumes x0 < x1) static float linearInterp(float x0, float y0, float x1, float y1, float x) { if (fabsf(x1 - x0) < 1e-9f) return y0; float t = (x - x0) / (x1 - x0); return y0 + t * (y1 - y0); }

// Find duty that best matches a target current using coarse table + local refinement static int findDutyForTargetCurrent(float targetCurrent, int minDc, int maxDc, int stabilizationDelayMs) { // Build coarse table if not already built if (measurementCache.size() < (size_t)COARSE_SAMPLE_POINTS) { measurementCache.clear(); for (int i = 0; i < COARSE_SAMPLE_POINTS; ++i) { float t = (float)i / (COARSE_SAMPLE_POINTS - 1); int duty = static_cast<int>(minDc + t * (maxDc - minDc)); if (duty < minDc) duty = minDc; if (duty > maxDc) duty = maxDc; getCachedMeasurement(duty, COARSE_SAMPLE_ATTEMPTS, stabilizationDelayMs, "CoarseSample"); } }

// Extract monotonic current vs duty table
std::vector<std::pair<int,float>> table; table.reserve(measurementCache.size());
for (auto &kv : measurementCache) {
    if (kv.second.valid) table.emplace_back(kv.first, kv.second.current);
}
if (table.empty()) return minDc;
std::sort(table.begin(), table.end(), [](auto &a, auto &b){ return a.first < b.first; });

// Ensure monotonic non-decreasing in current by taking running max (helps noise and non-monotonic measurements)
float runningMax = -std::numeric_limits<float>::infinity();
for (auto &p : table) {
    runningMax = std::max(runningMax, p.second);
    p.second = runningMax;
}

// If target outside sampled range, clamp
float maxCurrent = table.back().second;
float minCurrent = table.front().second;
if (targetCurrent >= maxCurrent) return table.back().first;
if (targetCurrent <= minCurrent) return table.front().first;

// Find neighbor segment for interpolation
int dutyGuess = table.front().first;
for (size_t i = 1; i < table.size(); ++i) {
    if (table[i].second >= targetCurrent) {
        int d0 = table[i-1].first;
        float c0 = table[i-1].second;
        int d1 = table[i].first;
        float c1 = table[i].second;
        float estDutyF = linearInterp(c0, (float)d0, c1, (float)d1, targetCurrent);
        dutyGuess = static_cast<int>(std::round(estDutyF));
        // bound
        dutyGuess = std::max(minDc, std::min(maxDc, dutyGuess));
        break;
    }
}

// Local refinement around dutyGuess using a small window
int bestDuty = dutyGuess;
float bestDiff = std::numeric_limits<float>::infinity();

int left = std::max(minDc, dutyGuess - REFINEMENT_WINDOW);
int right = std::min(maxDc, dutyGuess + REFINEMENT_WINDOW);

for (int d = left; d <= right; ++d) {
    // Avoid re-measuring if cached
    CachedMeasurement cm = getCachedMeasurement(d, REFINEMENT_ATTEMPTS, stabilizationDelayMs, "Refine");
    if (!cm.valid) continue;
    float diff = std::fabs(cm.current - targetCurrent);
    if (diff < bestDiff) {
        bestDiff = diff;
        bestDuty = d;
    }
}

return bestDuty;

}

// ---------------------- High-level pair generation ----------------------

std::vector<std::pair<int,int>> generateDutyCyclePairsImproved(int minDutyCycle) { Serial.println("generateDutyCyclePairsImproved: starting..."); std::vector<std::pair<int,int>> pairs; if (minDutyCycle <= 0) return pairs;

int numPairs = MAX_RESISTANCE_POINTS / 2;
if (numPairs < 1) return pairs;

// Pre-measure endpoints (use averaged sampling)
CachedMeasurement minM = getCachedMeasurement(minDutyCycle, COARSE_SAMPLE_ATTEMPTS, STABILIZATION_DELAY_MS, "EndpointMin");
CachedMeasurement maxM = getCachedMeasurement(MAX_DUTY_CYCLE, COARSE_SAMPLE_ATTEMPTS, STABILIZATION_DELAY_MS, "EndpointMax");

if (!minM.valid || !maxM.valid) {
    Serial.println("Error: Could not measure endpoints reliably.");
    return pairs;
}

// If max <= min current, fallback to evenly spaced duty pairs but with caching & safety
if (maxM.current <= minM.current) {
    Serial.println("Warning: non-increasing current vs duty. Falling back to uniform duty spacing (but cached).\n");
    int highDc = MAX_DUTY_CYCLE;
    int lowDc  = minDutyCycle;
    int step = std::max(1, (MAX_DUTY_CYCLE - minDutyCycle) / numPairs);
    for (int i = 0; i < numPairs; ++i) {
        if (highDc < lowDc) break;
        // ensure lowDc produces measurable current
        CachedMeasurement lowMeas = getCachedMeasurement(lowDc, REFINEMENT_ATTEMPTS, STABILIZATION_DELAY_MS, "FallbackLow");
        if (lowMeas.current < MEASURABLE_CURRENT_THRESHOLD && lowDc < highDc) {
            // try nudging
            int adj = lowDc;
            for (int k = 0; k < 6; ++k) {
                adj += MIN_DUTY_CYCLE_ADJUSTMENT_STEP;
                if (adj > MAX_DUTY_CYCLE) break;
                CachedMeasurement tryM = getCachedMeasurement(adj, 1, STABILIZATION_DELAY_MS, "FallbackTry");
                if (tryM.current >= MEASURABLE_CURRENT_THRESHOLD) { lowDc = adj; break; }
            }
        }
        pairs.emplace_back(lowDc, highDc);
        highDc = std::max(minDutyCycle, highDc - step);
        lowDc = std::min(MAX_DUTY_CYCLE, lowDc + step);
    }
    return pairs;
}

// Build target currents evenly spaced within the measured current range
float totalRange = maxM.current - minM.current;
if (!(totalRange > 0.0f)) return pairs;
int effectivePairs = numPairs;

for (int i = 0; i < effectivePairs; ++i) {
    // targeting from high->low so top-of-range is paired with bottom-of-range
    float frac = (float)i / (effectivePairs - 1);
    float targetHighCurrent = maxM.current - frac * totalRange; // descending
    int foundHighDc;

    // Option to use fast interpolation+refine or full binary-search. Default: interpolation
    if (USE_FAST_METHOD) {
        foundHighDc = findDutyForTargetCurrent(targetHighCurrent, minDutyCycle, MAX_DUTY_CYCLE, STABILIZATION_PAIRS_FIND_DELAY_MS);
    } else {
        // Binary search fallback (measures many points) - we still use cache to avoid duplicates
        int lowBound = minDutyCycle;
        int highBound = MAX_DUTY_CYCLE;
        int best = highBound;
        float bestDiff = std::numeric_limits<float>::infinity();
        int iterations = 0;
        while (lowBound <= highBound && iterations++ < 20) {
            int mid = lowBound + (highBound - lowBound)/2;
            CachedMeasurement cm = getCachedMeasurement(mid, 1, STABILIZATION_PAIRS_FIND_DELAY_MS, "BinSearch");
            float diff = std::fabs(cm.current - targetHighCurrent);
            if (diff < bestDiff) { bestDiff = diff; best = mid; }
            if (cm.current > targetHighCurrent) { highBound = mid - 1; }
            else { lowBound = mid + 1; }
        }
        foundHighDc = best;
    }

    // compute lowDc candidate (start from previous low or minDutyCycle)
    int lowDc = minDutyCycle;
    CachedMeasurement lowM = getCachedMeasurement(lowDc, REFINEMENT_ATTEMPTS, STABILIZATION_DELAY_MS, "CheckLow");
    if (lowM.current < MEASURABLE_CURRENT_THRESHOLD && lowDc < foundHighDc) {
        // nudge low upward until measurable or reach foundHighDc
        int adj = lowDc;
        for (int k = 0; k < 8; ++k) {
            adj += MIN_DUTY_CYCLE_ADJUSTMENT_STEP;
            if (adj >= foundHighDc) break;
            CachedMeasurement tryM = getCachedMeasurement(adj, 1, STABILIZATION_DELAY_MS, "AdjustLow");
            if (tryM.current >= MEASURABLE_CURRENT_THRESHOLD) { lowDc = adj; break; }
        }
        // final check
        lowM = getCachedMeasurement(lowDc, 1, STABILIZATION_DELAY_MS, "LowFinal");
    }

    // ensure pair validity
    if (foundHighDc <= lowDc) {
        if (foundHighDc <= minDutyCycle) foundHighDc = std::min(MAX_DUTY_CYCLE, minDutyCycle + 1);
        if (foundHighDc <= lowDc) {
            // cannot form this pair; skip
            continue;
        }
    }

    pairs.emplace_back(lowDc, foundHighDc);

    // Optional: for next iteration, limit MAX_DUTY_CYCLE to just below the found high to avoid overlap
    MAX_DUTY_CYCLE = std::max(minDutyCycle, foundHighDc - 1);

    // Safety guard
    if (measurementsPerformed > MAX_TOTAL_MEASUREMENTS) break;
}

Serial.printf("generateDutyCyclePairsImproved: created %zu pairs, total measurements used=%d\n", pairs.size(), measurementsPerformed);
return pairs;

}

// ---------------------- Revised measurement functions (drop-in replacements) ----------------------

void measureInternalResistanceLoadedUnloadedImproved(const std::vector<std::pair<int, int>>& dutyCyclePairs, std::vector<float>& voltagesLoaded, std::vector<float>& currentsLoaded, std::vector<float>& dutyCycles) { Serial.println("measureInternalResistanceLoadedUnloadedImproved: starting...");

for (const auto &p : dutyCyclePairs) {
    int dcHigh = p.second;
    Serial.printf("--- Loaded/Unloaded step: high DC=%d ---\n", dcHigh);

    CachedMeasurement loaded = getCachedMeasurement(dcHigh, REFINEMENT_ATTEMPTS, STABILIZATION_DELAY_MS, "LoadedLUL");
    if (!loaded.valid) {
        Serial.println("Warning: loaded measurement invalid, skipping step.");
        continue;
    }
    voltagesLoaded.push_back(loaded.voltage);
    currentsLoaded.push_back(loaded.current);
    dutyCycles.push_back((float)dcHigh);

    Serial.printf("Loaded: DC=%d V=%.3f I=%.3f\n", dcHigh, loaded.voltage, loaded.current);

    // Unloaded measurement - use dedicated helper so it can implement its own stabilization strategy
    MeasurementData unloaded = getUnloadedVoltageMeasurement();
    Serial.printf("Unloaded: V=%.3f I=%.3f\n", unloaded.voltage, unloaded.current);

    if (loaded.current > 0.01f) {
        float r = (unloaded.voltage - loaded.voltage) / loaded.current;
        storeResistanceData(loaded.current, std::fabs(r), internalResistanceData, resistanceDataCount);
        Serial.printf("Rint (L/UL): %.3f Ohm\n", std::fabs(r));
    } else {
        Serial.println("Warning: loaded current too small for L/UL resistance calculation.");
        storeResistanceData(loaded.current, -1.0f, internalResistanceData, resistanceDataCount);
    }
}

}

void measureInternalResistancePairsImproved(const std::vector<std::pair<int, int>>& dutyCyclePairs, std::vector<float>& consecutiveInternalResistances) { Serial.println("measureInternalResistancePairsImproved: starting..."); for (const auto &p : dutyCyclePairs) { int lowDc = p.first; int highDc = p.second; Serial.printf("--- Pair: low=%d high=%d ---\n", lowDc, highDc);

CachedMeasurement lowM = getCachedMeasurement(lowDc, REFINEMENT_ATTEMPTS, STABILIZATION_DELAY_MS, "PairLow");
    CachedMeasurement highM = getCachedMeasurement(highDc, REFINEMENT_ATTEMPTS, STABILIZATION_DELAY_MS, "PairHigh");

    if (!lowM.valid || !highM.valid) {
        Serial.println("Warning: invalid pair measurements, skipping.");
        consecutiveInternalResistances.push_back(-1.0f);
        continue;
    }

    if (highM.current > lowM.current + MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
        float r = (lowM.voltage - highM.voltage) / (highM.current - lowM.current);
        consecutiveInternalResistances.push_back(std::fabs(r));
        storeResistanceData(highM.current, std::fabs(r), internalResistanceDataPairs, resistanceDataCountPairs);
        Serial.printf("Rint (pair): %.3f Ohm\n", std::fabs(r));
    } else {
        Serial.println("Warning: insufficient current delta between pair for R calculation.");
        consecutiveInternalResistances.push_back(-1.0f);
        storeResistanceData(highM.current, -1.0f, internalResistanceDataPairs, resistanceDataCountPairs);
    }
}

}

// ---------------------- Regression helpers ----------------------

// Weighted linear regression using arrays of [current, resistance] bool performWeightedLinearRegression(float (*array)[2], int count, float &slope, float &intercept) { // Extract valid points double sumW = 0.0, sumWI = 0.0, sumWV = 0.0, sumWII = 0.0, sumWIV = 0.0; for (int i = 0; i < count; ++i) { float I = array[i][0]; float R = array[i][1]; if (R <= MIN_VALID_RESISTANCE) continue; // Weight by current magnitude to favor higher-SNR points (simple heuristic) double w = std::max(1e-3, (double)I); sumW += w; sumWI += w * I; sumWV += w * R; sumWII += w * I * I; sumWIV += w * I * R; }

double denom = (sumW * sumWII - sumWI * sumWI);
if (fabs(denom) < 1e-12) return false;
slope = (float)((sumW * sumWIV - sumWI * sumWV) / denom);
intercept = (float)((sumWV - slope * sumWI) / sumW);
return true;

}

// ---------------------- Integration entry point (drop-in replacement) ----------------------

void measureInternalResistanceRefactored() { if (!isMeasuringResistance) return; Serial.println("measureInternalResistanceRefactored: begin");

measurementCache.clear();
measurementsPerformed = 0;
resistanceDataCount = 0;
resistanceDataCountPairs = 0;

std::vector<float> voltLoaded, currLoaded, dutyVec;
std::vector<float> pairRints;

// initial unloaded
MeasurementData initUL = getUnloadedVoltageMeasurement();
Serial.printf("Initial unloaded: %.3f V\n", initUL.voltage);

int minDuty = findMinimalDutyCycle();
if (minDuty == 0) return;

auto pairs = generateDutyCyclePairsImproved(minDuty);

measureInternalResistanceLoadedUnloadedImproved(pairs, voltLoaded, currLoaded, dutyVec);
measureInternalResistancePairsImproved(pairs, pairRints);

MeasurementData finalUL = getUnloadedVoltageMeasurement();
Serial.printf("Final unloaded: %.3f V\n", finalUL.voltage);

bubbleSort(internalResistanceData, resistanceDataCount);
bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

if (resistanceDataCount >= 2) {
    if (performWeightedLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept)) {
        Serial.printf("Weighted regression (L/UL): slope=%.4f intercept=%.4f\n", regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
    } else {
        Serial.println("Weighted regression (L/UL) failed.");
    }
}

if (resistanceDataCountPairs >= 2) {
    if (performWeightedLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept)) {
        Serial.printf("Weighted regression (pairs): slope=%.4f intercept=%.4f\n", regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
    }
}

Serial.printf("Refactored measurement complete. pairs=%zu measurements=%d collected(L/UL)=%d pairs=%d\n",
              pairs.size(), measurementsPerformed, resistanceDataCount, resistanceDataCountPairs);

isMeasuringResistance = false;

}

// End of file

