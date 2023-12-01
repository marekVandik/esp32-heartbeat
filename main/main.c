#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "hal/adc_types.h"
#include "sdkconfig.h"
#include "ssd1306.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"


// Defines:
#define DISPLAY_HEIGHT 64
#define DISPLAY_WIDTH 128
#define INVERTED true
#define NOT_INVERTED false
#define HIGH_CONTRAST 0xFF
#define LOW_CONTRAST 0
#define ADC_FRAME_SIZE 256
#define ADC_POOL_SIZE 1024
#define CLEAR_AHEAD 5


// Constants:
const static char *TAG = "imp-project";
const static char *AVERAGING_TAG = "ADC averaging task";
const static uint8_t empty_buffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8] = {};


// Global vars
static TaskHandle_t adc_avg_results_task_handle = NULL;
static uint16_t adc_avg = 0;
static adc_continuous_handle_t adc_handle = NULL;


// Functions:
/**
 * @brief Adc conversion frame averaging task, stores avg in adc_avg global var
 * 
 * @param pvParams not used
 */
void adc_avg_results_task(void * pvParams) {
    uint8_t conversion_frame_buffer[ADC_FRAME_SIZE];
    uint32_t acc;
    uint32_t counter;
    uint32_t read_bytes_length;
    esp_err_t read_result;

    while (1) {
        // yields until notified by adc_notify_callback or portMAX_DELAY timeout
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        acc = 0;
        counter = 0;

        // averaging loop runs until there is no more data from the ADC
        while (1) {
            // reads up to ADC_FRAME_SIZE of data into conversion_frame_buffer
            read_result = adc_continuous_read(adc_handle, conversion_frame_buffer, ADC_FRAME_SIZE, &read_bytes_length, 0);
            if (read_result == ESP_OK) {
                // accumulates values into acc accumulator and increments counter across one conversion frame
                for (uint32_t i = 0; i < read_bytes_length; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *data = (void *) &conversion_frame_buffer[i];
                    if (data->type1.channel == 0) {
                        acc += data->type1.data;
                        counter++;
                    }
                }
            } else if (read_result == ESP_ERR_TIMEOUT) {
                // there is no data to be read currently
                break;
            }
        }

        // computes the average and saves it into glob var adc_avg
        if (counter != 0) {
            adc_avg = acc / counter;
        }
    }
}

/**
 * @brief ADC conversion callback, this gets called when the conversion frame is full and notifies the ADC averaging task above
 * 
 */
 bool adc_notify_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t must_yield = pdFALSE;
    vTaskNotifyGiveFromISR(adc_avg_results_task_handle, &must_yield);

    return (must_yield == pdTRUE);
 }

/**
 * @brief Configures GPIO port 36 (ADC channel 0) to serve as input and configures ADC to use channel 0 with specified parameters
 * 
 */
void adc_init(adc_continuous_handle_t *handle) {
    ESP_LOGI(TAG, "Creating ADC handle with %dB frame size and %dB internal pool", ADC_FRAME_SIZE, ADC_POOL_SIZE);
    adc_continuous_handle_cfg_t handle_config = {
        .max_store_buf_size = ADC_POOL_SIZE,
        .conv_frame_size = ADC_FRAME_SIZE
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, handle));

    ESP_LOGI(TAG, "Configuring ADC channel 0 with 0 dB attenuation, using ADC unit 1 and result bit size 9");
    adc_digi_pattern_config_t digi_config = {
        .atten = ADC_ATTEN_DB_0,
        .channel = ADC_CHANNEL_0,   // GPIO36
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12
    };

    ESP_LOGI(TAG, "Configuring ADC to use channel 0 with before mentioned specs and on 20kHz sampling frequency using single conversion unit");
    adc_continuous_config_t config = {
        .pattern_num = 1,
        .adc_pattern = &digi_config,
        .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_LOW,    // 20kHz
        .conv_mode = ADC_CONV_SINGLE_UNIT_1
    };

    ESP_ERROR_CHECK(adc_continuous_config(*handle, &config));

    ESP_LOGI(TAG, "Registering callbacks for ADC events");
    adc_continuous_evt_cbs_t callbacks = {
        .on_conv_done = adc_notify_callback,
        .on_pool_ovf = NULL
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(*handle, &callbacks, NULL));

    ESP_LOGI(TAG, "Starting ADC");
    ESP_ERROR_CHECK(adc_continuous_start(*handle));
}

void create_adc_avg_task(void) {
    ESP_LOGI(TAG, "Creating task - %s", AVERAGING_TAG);
    xTaskCreate(adc_avg_results_task, AVERAGING_TAG, 1024, NULL, 0, &adc_avg_results_task_handle);
}

/**
 * @brief Sets up I2C interface to communicate with ssd1306 componentand instantiates display device handle with correct parameters clearing the screen
 *
 */
void display_init(SSD1306_t *display_handle) {
    ESP_LOGI(TAG, "Configuring ssd1306 over I2C bus:");
    ESP_LOGI(TAG, "\tCONFIG_SDA_GPIO - %d", CONFIG_SDA_GPIO);
    ESP_LOGI(TAG, "\tCONFIG_SCL_GPIO - %d", CONFIG_SCL_GPIO);
    ESP_LOGI(TAG, "\tCONFIG_RESET_GPIO - %d", CONFIG_RESET_GPIO);
    i2c_master_init(display_handle, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

    ESP_LOGI(TAG, "Instantiating ssd1306 panel with %dx%d pixels",
        DISPLAY_WIDTH, DISPLAY_HEIGHT
    );
    ssd1306_init(display_handle, DISPLAY_WIDTH, DISPLAY_HEIGHT);

    ESP_LOGI(TAG, "Setting contrast on ssd1306 to high");
    ssd1306_contrast(display_handle, HIGH_CONTRAST);

    ESP_LOGI(TAG, "Clearing display contents (random noise)");
    ssd1306_clear_screen(display_handle, NOT_INVERTED);
}

/**
 * @brief Updates the display using the current adc_avg placing next pixel
 * 
 * @param display_device reference to ssd1306 device
 */
void display_update(SSD1306_t *display_device) {
    static uint8_t data_rows_buffer[DISPLAY_WIDTH] = {0};
    static uint16_t data_buffer[DISPLAY_WIDTH] = {0};
    static uint8_t data_idx = 0;

    static uint16_t max;
    static uint16_t min;
    static uint16_t distance_per_row;

    if (data_idx == 0) {
        max = 0;
        min = INT16_MAX;
    }

    uint16_t current_adc_avg = adc_avg;
    data_buffer[data_idx] = current_adc_avg;

    if (current_adc_avg > max) {
        max = current_adc_avg;
    }
    if (current_adc_avg < min) {
        min = current_adc_avg;
    }
    distance_per_row = (max - min) / DISPLAY_HEIGHT;

    if (distance_per_row == 0) {
        // arbitrary value to continue drawing
        distance_per_row = 1;
    }

    // recalculates pixel row positions
    for (uint8_t i = 0; i < DISPLAY_WIDTH; i++) {
        uint8_t calculated_row = (max - data_buffer[i]) / distance_per_row;
        
        if (calculated_row >= DISPLAY_HEIGHT) {
            calculated_row = DISPLAY_HEIGHT - 1;
        }

        data_rows_buffer[i] = calculated_row;
    }

    ssd1306_set_buffer(display_device, empty_buffer);

    // renders data into bitmap up to current index clearing CLEAR_AHEAD pixels in front of current index
    // Note: renders in rows to ensure best memory locality
    for (uint8_t row_top = 0; row_top < DISPLAY_HEIGHT; row_top++) {
        // placing pixels into correct rows
        for (uint8_t col_left = 0; col_left <= data_idx; col_left++) {
            if (data_rows_buffer[col_left] == row_top) {
                // place pixel into correct position
                _ssd1306_pixel(display_device, col_left, row_top, NOT_INVERTED);
            }
        }

        // clearing X pixels ahead
        for (uint8_t clear_idx = data_idx + 1; (clear_idx <= data_idx + CLEAR_AHEAD) && (clear_idx < DISPLAY_WIDTH); clear_idx++) {
            _ssd1306_pixel(display_device, clear_idx, row_top, INVERTED);
        }
    }

    ESP_LOGI(TAG, "Printing bitmap");
    ssd1306_show_buffer(display_device);

    data_idx = (data_idx + 1) % DISPLAY_WIDTH;
} 

void app_main(void) {
    SSD1306_t display_device;
    display_init(&display_device);

    create_adc_avg_task();

    adc_init(&adc_handle);

    // prints hello on the display
    ssd1306_display_text(&display_device, 0, "Hello!", 6, false);

    while (1) {
        ESP_LOGI(TAG, "adc_avg: %d", adc_avg);
        display_update(&display_device);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
