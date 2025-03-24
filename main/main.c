#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "ssd1306.h"
#include "gfx.h"

#define SSD1306_SPI
#define TRIGGER_PIN 16
#define ECHO_PIN    17

QueueHandle_t xQueueTime;
QueueHandle_t xQueueDistance;

void pin_callback(uint gpio, uint32_t events) {
    static uint64_t start_time = 0;
    static uint64_t end_time   = 0;

    if (events & GPIO_IRQ_EDGE_RISE) {
        start_time = time_us_64();
    } 
    else if (events & GPIO_IRQ_EDGE_FALL) {
        end_time = time_us_64();
        uint64_t delta_t = end_time - start_time;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(xQueueTime, &delta_t, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void trigger_task(void *pvParameters) {
    while (1) {
        gpio_put(TRIGGER_PIN, 0);
        sleep_us(2);
        gpio_put(TRIGGER_PIN, 1);
        sleep_us(10);
        gpio_put(TRIGGER_PIN, 0);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void echo_task(void *pvParameters) {
    uint64_t delta_t;
    float distance;

    while (1) {
        if (xQueueReceive(xQueueTime, &delta_t, portMAX_DELAY) == pdTRUE) {
            distance = (delta_t * 0.0343f) / 2.0f;

            if (delta_t < 50 || delta_t > 30000) {
                distance = -1.0f;
            }
            xQueueSend(xQueueDistance, &distance, 0);
        }
    }
}

void oled_task(void *pvParameters) {
    ssd1306_t disp;
    ssd1306_init();
    gfx_init(&disp, 128, 32);

    float distance;
    char buffer[20];
    int failure_count = 0;
    const int FAILURE_THRESHOLD = 3;

    while (1) {
        if (xQueueReceive(xQueueDistance, &distance, portMAX_DELAY) == pdTRUE) {
            gfx_clear_buffer(&disp);
            if (distance < 0) {
                failure_count++;
                if (failure_count >= FAILURE_THRESHOLD) {
                    gfx_draw_string(&disp, 0, 0, 1, "Sensor Falhou");
                } else {
                    gfx_draw_string(&disp, 0, 0, 1, "Falha na leitura");
                }
            } else {
                failure_count = 0;
                sprintf(buffer, "Dist: %.1f cm", distance);
                gfx_draw_string(&disp, 0, 0, 1, buffer);

                int bar_width = (int)((distance / 100.0f) * 128);
                if (bar_width > 128) bar_width = 128;
                gfx_draw_line(&disp, 0, 20, bar_width, 20);
            }
            gfx_show(&disp);
        }
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_put(TRIGGER_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_set_pulls(ECHO_PIN, false, true);

    // Configura interrupções no pino de ECHO para detecção de subida e descida
    gpio_set_irq_enabled_with_callback(
        ECHO_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        pin_callback
    );

    // Cria as filas
    xQueueTime = xQueueCreate(5, sizeof(uint64_t));
    xQueueDistance = xQueueCreate(5, sizeof(float));

    // Cria as tasks
    xTaskCreate(trigger_task, "TriggerTask", 256, NULL, 1, NULL);
    xTaskCreate(echo_task,    "EchoTask",    256, NULL, 1, NULL);
    xTaskCreate(oled_task,    "OLEDTask",   4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1) {
    }
    return 0;
}
