/**
 * @file furi_hal_speaker.c
 * Speaker HAL (ESP32 stub - no-op, no speaker hardware)
 */

#include "furi_hal_speaker.h"

#include <kernel.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <board.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "sdkconfig.h"

#define I2S_SAMPLE_RATE 16000
#define I2S_BUFF_SIZE 320
#define I2S_LUT_SIZE 320

static float shared_frequency = 440.0;
static float shared_volume = 1.0;
static TaskHandle_t i2s_write_task_handle;
static pthread_mutex_t mutex;
static int lock_result = -1;
static i2s_chan_handle_t tx_chan;
static int16_t LUT[I2S_LUT_SIZE];      // our sine wave LUT
static int16_t buff[I2S_BUFF_SIZE];

static const char* TAG = "Speaker";

static void i2s_write_task(void* args)
{
    pthread_mutex_init(&mutex, NULL);
    

    for (int i = 0; i < I2S_LUT_SIZE; ++i)
    {
        LUT[i] = (int16_t)roundf(SHRT_MAX * sinf(2.0f * M_PI * (float)i / I2S_LUT_SIZE));
    }

    size_t w_bytes = I2S_BUFF_SIZE;

    while (1) {
        const float delta_phi = (float) shared_frequency / (float) I2S_SAMPLE_RATE * (float) I2S_LUT_SIZE;
        float phase = 0.0f;

        for (int i = 0; i < I2S_BUFF_SIZE; i++) { 
        int phase_i = (int)phase;
            buff[i] = LUT[phase_i]; 
            buff[i] = (int16_t)((float)buff[i]) * shared_volume;
            phase += delta_phi;
            if (phase >= (float)I2S_LUT_SIZE)    // handle wrap around
                phase -= (float)I2S_LUT_SIZE;
        }

        if (i2s_channel_write(tx_chan, buff, I2S_BUFF_SIZE * sizeof(int16_t), &w_bytes, 20) != ESP_OK)
        {
            ESP_LOGE(TAG, "i2s write failed");
        }
        else 
        {
            //ESP_LOGI(TAG, "wrote %d bytes", w_bytes);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelete(NULL);
}

void furi_hal_speaker_init(void) {
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

    i2s_std_config_t tx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    // some codecs may require mclk signal, this example doesn't need it
            .bclk = BOARD_PIN_SPEAKER_BCLK,
            .ws   = BOARD_PIN_SPEAKER_WCLK,
            .dout = BOARD_PIN_SPEAKER_DOUT,
            .din = I2S_GPIO_UNUSED,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));

    BaseType_t xResult = xTaskCreate(i2s_write_task, "i2s_write_task", 4096, NULL, 5, &i2s_write_task_handle);
    if (xResult == pdPASS)
    {
        vTaskSuspend(i2s_write_task_handle);
    }
}

void furi_hal_speaker_deinit(void) {
    i2s_del_channel(tx_chan);
    pthread_mutex_destroy(&mutex);
}

bool furi_hal_speaker_acquire(uint32_t timeout) {
    struct timespec ttimeout;
    ttimeout.tv_nsec = timeout * 1000000;
    if (lock_result == 0) {
        return true;
        // we're already holding the lock!
    }
    else 
    {
        lock_result = pthread_mutex_timedlock(&mutex, &ttimeout);
        return lock_result == 0;
    }
}

void furi_hal_speaker_release(void) {
    pthread_mutex_unlock(&mutex);
    lock_result = -1;
}

bool furi_hal_speaker_is_mine(void) {
    return lock_result == 0;
}

void furi_hal_speaker_start(float frequency, float volume) {
    //ESP_LOGI(TAG, "starting speaker, frequency: %.2f, volume: %.2f", frequency, volume);
    shared_frequency = frequency;
    shared_volume = volume;

    i2s_channel_enable(tx_chan);
    vTaskResume(i2s_write_task_handle);

}

void furi_hal_speaker_set_volume(float volume) {
    //ESP_LOGI(TAG, "setting volume: %.2f", volume);
    shared_volume = volume;
}

void furi_hal_speaker_stop(void) {
    //ESP_LOGI(TAG, "stopping speaker");
    vTaskSuspend(i2s_write_task_handle);
    i2s_channel_disable(tx_chan);
}


