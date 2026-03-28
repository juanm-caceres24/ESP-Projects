#include <Arduino.h>

// Default values used on startup.
const uint32_t DEFAULT_ITERATIONS = 70460000;
const uint32_t CPU_FREQ_MHZ[] = {80, 160, 240};
const size_t CPU_FREQ_COUNT = sizeof(CPU_FREQ_MHZ) / sizeof(CPU_FREQ_MHZ[0]);
// Volatile counter to prevent compiler optimizations.
volatile uint32_t counter = 0;
volatile uint32_t requested_freq_mhz = CPU_FREQ_MHZ[0];
volatile uint32_t requested_iterations = DEFAULT_ITERATIONS;
volatile unsigned long long start_time_us = 0;
volatile unsigned long long end_time_us = 0;
volatile unsigned long long total_time_us = 0;
volatile bool freq_ok = false;

static void runBenchmark(uint32_t freqMHz, uint32_t iterations) {
    requested_freq_mhz = freqMHz;
    requested_iterations = iterations;
    setCpuFrequencyMhz(freqMHz);
    counter = 0;
    start_time_us = micros();
    for (uint32_t i = 0; i < iterations; i++) {
        counter++;
    }
    end_time_us = micros();
    total_time_us = end_time_us - start_time_us;
}

static void printStats() {
    setCpuFrequencyMhz(240);
    Serial.printf("CPU Frequency: %lu MHz\n", (unsigned long)requested_freq_mhz);
    Serial.printf("Total Time: %llu [us] (%.3f [ms])\n", total_time_us, (double)total_time_us / 1000.0);
    Serial.printf("Final Counter: %lu\n", (unsigned long)counter);
}

static void runAllBenchmarks() {
    setCpuFrequencyMhz(240);
    Serial.printf("Running %lu iterations for each supported frequency...\n", (unsigned long)DEFAULT_ITERATIONS);
    for (size_t i = 0; i < CPU_FREQ_COUNT; i++) {
        runBenchmark(CPU_FREQ_MHZ[i], DEFAULT_ITERATIONS);
        printStats();
    }
}

void setup() {
    setCpuFrequencyMhz(240);
    Serial.begin(115200);
    delay(1000);
    Serial.println("ESP32 automatic time benchmark");
    runAllBenchmarks();
}

void loop() {
    delay(1000);
}
