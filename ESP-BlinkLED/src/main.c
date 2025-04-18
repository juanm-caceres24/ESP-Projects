#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_BUILTIN 2
#define GPIO_MODE_OUTPUT 1

void app_main() {
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(LED_BUILTIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_BUILTIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}