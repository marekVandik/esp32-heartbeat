#include <stdio.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "ssd1306.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DISPLAY_HEIGHT 64
#define DISPLAY_WIDTH 128


const static char *TAG = "imp-project";

/**
 * @brief Sets up I2C interface to communicate with ssd1306 component
 and instantiates display device handle with correct parameters
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
    ssd1306_contrast(display_handle, 0xFF); // 0xFF - high, 0 - low
}

void app_main(void) {
    SSD1306_t display_device;
    display_init(&display_device);

    // TODO gpio config for ADC channel 0

    // prints hello on the display
    ssd1306_display_text(&display_device, 0, "Hello!", 6, false);
}
