#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

#define PIR_GPIO   GPIO_NUM_27
#define LED_GPIO   GPIO_NUM_25
#define SERVO_GPIO GPIO_NUM_33   // <-- your servo signal pin

#define HOLD_MS 500

static const char *TAG = "RTOS";

SemaphoreHandle_t motionSemaphore;

/* ---------- SERVO CONFIG ---------- */
#define SERVO_FREQ_HZ 50
#define SERVO_TIMER   LEDC_TIMER_0
#define SERVO_MODE    LEDC_LOW_SPEED_MODE
#define SERVO_CHANNEL LEDC_CHANNEL_0
#define SERVO_RES     LEDC_TIMER_16_BIT

#define SERVO_MIN_US  1000
#define SERVO_MAX_US  2000

static void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = SERVO_MODE,
        .timer_num        = SERVO_TIMER,
        .duty_resolution  = SERVO_RES,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel    = SERVO_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&channel);
}

static uint32_t us_to_duty(uint32_t us)
{
    uint32_t period_us = 1000000 / SERVO_FREQ_HZ;
    uint32_t max_duty = (1 << SERVO_RES) - 1;
    return (uint32_t)(((uint64_t)us * max_duty) / period_us);
}

static void servo_set_angle(int angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint32_t pulse = SERVO_MIN_US +
                     ((SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;

    uint32_t duty = us_to_duty(pulse);

    ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);
}

/* ---------- LED + SERVO TASK ---------- */
void led_task(void *pv)
{
    int64_t last_motion_us = 0;
    int state = 0;

    while (1) {
        if (xSemaphoreTake(motionSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            last_motion_us = esp_timer_get_time();
        }

        int64_t now = esp_timer_get_time();
        int active = (last_motion_us != 0) &&
                     ((now - last_motion_us) < (HOLD_MS * 1000));

        if (active != state) {
            state = active;
            gpio_set_level(LED_GPIO, state);

            if (state) {
                ESP_LOGI(TAG, "LED ON → Servo to 90°");
                servo_set_angle(90);
            } else {
                ESP_LOGI(TAG, "LED OFF → Servo to 0°");
                servo_set_angle(0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---------- PIR TASK ---------- */
void pir_task(void *pv)
{
    int last = 0;

    while (1) {
        int pir = gpio_get_level(PIR_GPIO);

        if (pir && !last) {
            xSemaphoreGive(motionSemaphore);
        }

        last = pir;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---------- MAIN ---------- */
void app_main(void)
{
    gpio_config_t pir = {
        .pin_bit_mask = 1ULL << PIR_GPIO,
        .mode = GPIO_MODE_INPUT
    };
    gpio_config(&pir);

    gpio_config_t led = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&led);
    gpio_set_level(LED_GPIO, 0);

    servo_init();
    servo_set_angle(0);

    motionSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(led_task, "LED+SERVO", 3072, NULL, 2, NULL);
    xTaskCreate(pir_task, "PIR", 2048, NULL, 3, NULL);

    ESP_LOGI(TAG, "System ready. PIR warm-up 30–60s.");
}
