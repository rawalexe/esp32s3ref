#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_driver.h"
#include "esp_idf_version.h"
//#include <Adafruit_NeoPixel.h>

static const char *TAG = "app_driver";

// config this through menuconfig
//#define GRI_LED_GPIO CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_GPIO_NUMBER
//#define GRI_LED_GPIO 21 # for xiao onboard
#define GRI_LED_GPIO 44

#ifdef CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT
//ESP_LOGE(TAG, "CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT is defined");

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    //ESP_LOGE(TAG, "CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT led_strip 1");
    static led_strip_handle_t led_strip;
#else
    static led_strip_t *led_strip;
    //ESP_LOGE(TAG, "CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT led_strip 1");
#endif
#endif

#if ESP_IDF_VERSION == ESP_IDF_VERSION_VAL(4, 3, 0)
#ifndef CONFIG_IDF_TARGET_ESP32
#define APP_SOC_TEMP_SENSOR_SUPPORTED
#else
#define APP_SOC_TEMP_SENSOR_SUPPORTED SOC_TEMP_SENSOR_SUPPORTED
#endif
#endif

static esp_err_t temperature_sensor_init()
{
#ifdef APP_SOC_TEMP_SENSOR_SUPPORTED
    // Initialize touch pad peripheral, it will start a timer to run a filter
    ESP_LOGI(TAG, "Initializing Temperature sensor");
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor_get_config(&temp_sensor);
    ESP_LOGD(TAG, "default dac %d, clk_div %d", temp_sensor.dac_offset, temp_sensor.clk_div);
    temp_sensor.dac_offset = TSENS_DAC_DEFAULT; // DEFAULT: range:-10℃ ~  80℃, error < 1℃.
    temp_sensor_set_config(temp_sensor);
    return(temp_sensor_start());
#else
    /* For the SoCs that do not have a temperature sensor (like ESP32) we report a dummy value. */
    return ESP_OK;
#endif
}

static esp_err_t led_init()
{
    esp_err_t ret = ESP_FAIL;

#ifdef CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT
ESP_LOGE(TAG, "CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT is defined 2");
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    led_strip_config_t strip_config = {
        .strip_gpio_num = GRI_LED_GPIO,
        .max_leds = 16, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
#else
    led_strip = led_strip_init(0, GRI_LED_GPIO, 1);
#endif

#elif CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_GPIO
    ret  = gpio_reset_pin(GRI_LED_GPIO);
    ret |= gpio_set_direction(GRI_LED_GPIO, GPIO_MODE_OUTPUT);

#endif
    
    //ret |= app_driver_led_on();
    return ret;
}

esp_err_t app_driver_init()
{
    esp_err_t temp_sensor_ret, led_ret;
	temp_sensor_ret = temperature_sensor_init();
    led_ret = led_init();
    if (temp_sensor_ret && led_ret) {
            return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

// #include "esp_rom_gpio.h"
// #include <driver/gpio.h>
// #define LED_PIN 2

// void led_blink(void *pvParams) {
//     // gpio_pad_select_gpio(LED_PIN);
//     esp_rom_gpio_pad_select_gpio(LED_PIN);
//     gpio_set_direction (LED_PIN,GPIO_MODE_OUTPUT);
//     while (1) {
//         gpio_set_level(LED_PIN,0);
//         vTaskDelay(1000/portTICK_PERIOD_MS);
//         gpio_set_level(LED_PIN,1);
//         vTaskDelay(1000/portTICK_PERIOD_MS);
//     }
// }

esp_err_t app_driver_led_on(uint32_t r,uint32_t g, uint32_t b)
{
    esp_err_t ret = ESP_FAIL;

#ifdef CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    //ESP_LOGE( TAG, "app_driver_led_on : ON" );
    // ret  = led_strip_set_pixel(led_strip, 0, 0, 25, 0);
    // ret  = led_strip_set_pixels(led_strip, 0, 15, 0);
    for (size_t i = 0; i < 16; i++)
        led_strip_set_pixel(led_strip, i, r, g, b);
        
    /* Refresh the strip to send data */
    ret |= led_strip_refresh(led_strip);

    // for Seeed Xiao
    //ret = gpio_set_level(GRI_LED_GPIO, 0);
    gpio_set_level(21, 0);
#else
    ESP_LOGE( TAG, "app_driver_led_on : 2" );
    led_strip->set_pixel(led_strip, 0, 0, 25, 0);
    led_strip->refresh(led_strip, 100);
#endif

#elif CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_GPIO
    ESP_LOGE( TAG, "app_driver_led_on : 3" );
    ret = gpio_set_level(GRI_LED_GPIO, 0);

#endif
    return ret;
}

esp_err_t app_driver_led_off()
{
    esp_err_t ret = ESP_FAIL;

#ifdef CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ret = led_strip_clear(led_strip);
    // ESP_LOGE( TAG, "app_driver_led_off : OFF" );
    // for Seeed Xiao
    //ret = gpio_set_level(GRI_LED_GPIO, 0);
    gpio_set_level(21, 1);    
#else
    //ESP_LOGE( TAG, "app_driver_led_off : OFF - #2" );
    led_strip->clear(led_strip, 50);
#endif

#elif CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_GPIO
    ESP_LOGE( TAG, "app_driver_led_off : 1" );
    ESP_LOGE( TAG, "app_driver_led_off : 1 %i", GRI_LED_GPIO);
    ret = gpio_set_level(GRI_LED_GPIO, 1);

#endif
    return ret;
}

float app_driver_temp_sensor_read_celsius()
{
#ifdef APP_SOC_TEMP_SENSOR_SUPPORTED
    float tsens_out;
    temp_sensor_read_celsius(&tsens_out);
    return tsens_out;
#else
    /* For the SoCs that do not have a temperature sensor (like ESP32) we report a dummy value. */
    return 0.0f;
#endif
}
