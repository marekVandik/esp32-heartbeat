#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "hal/adc_types.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"

#include "ssd1306.h"
#include "fft.h"


// Defines:
#define DISPLAY_HEIGHT 64
#define DISPLAY_WIDTH 128
#define INVERTED true
#define NOT_INVERTED false
#define HIGH_CONTRAST 0xFF
#define LOW_CONTRAST 0
#define SAMPLING_FREQ 20000
#define ADC_FRAME_SIZE (SOC_ADC_DIGI_RESULT_BYTES * 16)     // 16 samples at 20kHz ~ 1250Hz averaged multi-sampling frequency
#define ADC_POOL_SIZE 1024                                  // ADC driver buffer size
#define ADC_MULTISAMPLED_BUFFER_SIZE 2048                   // FFT accuracy ~0.3Hz when multi-sampled results are available at 625Hz
#define CLEAR_AHEAD 5


// Constants:
const static char *TAG = "imp-project";
const static char *AVERAGING_TAG = "ADC averaging task";
const static uint8_t empty_buffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8] = {0};


// Global vars
static TaskHandle_t adc_avg_results_task_handle = NULL;
static adc_continuous_handle_t adc_handle = NULL;
static uint16_t adc_averaged_results[ADC_MULTISAMPLED_BUFFER_SIZE] = {0};
static uint16_t adc_idx = 0;

static float fft_input[ADC_MULTISAMPLED_BUFFER_SIZE];
static float fft_output[ADC_MULTISAMPLED_BUFFER_SIZE * 2];
static fft_config_t *fft_handle = NULL;


// Functions:
/**
 * @brief Increments index to circular buffer adc_averaged_results
 */
void adc_idx_increment(void) {
    adc_idx = (adc_idx + 1) % ADC_MULTISAMPLED_BUFFER_SIZE;
}

/**
 * @brief Returns current adc sample from circular buffer adc_averaged_results
 * 
 * @return Currently selected adc sample
 */
uint16_t adc_buffer_get(void) {
    return adc_averaged_results[adc_idx];
}

/**
 * @brief Sets current adc sample in circular buffer adc_averaged_results
 * 
 * @param val Value to set
 */
void adc_buffer_set(uint16_t val) {
    adc_averaged_results[adc_idx] = val;
}

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

        // averaging loop runs until there is no more data from the ADC
        while (1) {
            // reads up to ADC_FRAME_SIZE of data into conversion_frame_buffer
            read_result = adc_continuous_read(adc_handle, conversion_frame_buffer, ADC_FRAME_SIZE, &read_bytes_length, 0);
            if (read_result == ESP_OK) {
                acc = 0;
                counter = 0;

                // accumulates values into acc accumulator and increments counter across one conversion frame
                for (uint32_t i = 0; i < read_bytes_length; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *data = (void *) &conversion_frame_buffer[i];
                    if (data->type1.channel == 0) {
                        acc += data->type1.data;
                        counter++;
                    }
                }
                adc_idx_increment();
                adc_buffer_set(counter != 0 ? (acc / counter) : 0);
            } else if (read_result == ESP_ERR_TIMEOUT) {
                // there is no data to be read currently
                break;
            }
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

    ESP_LOGI(TAG, "Configuring ADC channel 0 with 0 dB attenuation, using ADC unit 1 and result bit size 12");
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
        .sample_freq_hz = SAMPLING_FREQ,
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
    xTaskCreate(adc_avg_results_task, AVERAGING_TAG, 2048, NULL, 0, &adc_avg_results_task_handle);
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

    uint16_t current_adc_avg = adc_buffer_get();
    data_buffer[data_idx] = current_adc_avg;

    // finds max and min values in the data buffer
    max = data_buffer[0];
    min = data_buffer[0];
    for (uint8_t i = 1; i < DISPLAY_WIDTH; i++) {
        uint16_t val = data_buffer[i];
        if (val > max) {
            max = val;
        }
        if (val < min) {
            min = val;
        }
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
    for (uint8_t column = 0; column < DISPLAY_WIDTH; column++) {
        _ssd1306_pixel(display_device, column, data_rows_buffer[column], NOT_INVERTED);
    }
    for (uint8_t clear = 1; clear <= CLEAR_AHEAD; clear++) {
        uint8_t position_to_clear = (data_idx + clear) % DISPLAY_WIDTH;
        _ssd1306_pixel(display_device, position_to_clear, data_rows_buffer[position_to_clear], INVERTED);
    }

    ssd1306_show_buffer(display_device);

    data_idx = (data_idx + 1) % DISPLAY_WIDTH;
} 

void compute_fft(void) {
    TickType_t start = xTaskGetTickCount();
    uint16_t head_idx = (adc_idx + 1) % ADC_MULTISAMPLED_BUFFER_SIZE;

    // converts all data into floats
    for (uint16_t i = 0; i < ADC_MULTISAMPLED_BUFFER_SIZE; i++) {
        fft_input[i] = (float) (adc_averaged_results[(head_idx + i) % ADC_MULTISAMPLED_BUFFER_SIZE]);
    }
    
    fft_execute(fft_handle);

    static float magnitudes[ADC_MULTISAMPLED_BUFFER_SIZE] = {0};
    for (int k = 1; k < ADC_MULTISAMPLED_BUFFER_SIZE; k++) {
        magnitudes[k] = sqrtf(powf(fft_output[2*k], 2.0) + powf(fft_output[2*k + 1], 2.0));
    }
    float max = 0.0;
    int section = 0;
    for (int i = 0; i < ADC_MULTISAMPLED_BUFFER_SIZE; i++) {
        if (max < magnitudes[i]) {
            max = magnitudes[i];
            section = i;
        }
    }

    TickType_t end = xTaskGetTickCount();
    ESP_LOGI(TAG, "initialising fft took: %d ticks", (int) (end - start));
    ESP_LOGI(TAG, "max magnitude is %f at %d which is %f Hz", max, section, 625 * ((float)section / ADC_MULTISAMPLED_BUFFER_SIZE) / 2.0f);

    //fft_destroy(config);
}

void app_main(void) {
    SSD1306_t display_device;
    display_init(&display_device);

    create_adc_avg_task();

    adc_init(&adc_handle);

    fft_handle = fft_init(ADC_MULTISAMPLED_BUFFER_SIZE, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

    while (1) {
        compute_fft();
        display_update(&display_device);
        //vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
