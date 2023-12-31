// Thanks to Roger Chengs StepperVelocity for most of the useful code.
// Standard MIT License etc etc

// PWM generation code adapted from Espressif ESP-IDF LEDC PWM example
// https://github.com/espressif/esp-idf/blob/master/examples/peripherals/ledc/main/ledc_example_main.c

// Digital output code extracted from Espressif ESP-IDF GPIO example
// https://github.com/espressif/esp-idf/blob/master/examples/peripherals/gpio/generic_gpio/main/gpio_example_main.c

// Potentiometer reading code extracted from Espressif ESP-IDF ADC1 example
// https://github.com/espressif/esp-idf/blob/master/examples/peripherals/adc/main/adc1_example_main.c

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include <hd44780.h>
#include <esp_idf_lib_helpers.h>
#include "i2cdev.h"
#include <pcf8574.h>
#include <string.h>
#include <button.h>



// STEP and DIRECTION output pins for stepper motor driver.
static const gpio_num_t step_pin = GPIO_NUM_13;
static const gpio_num_t direction_pin = GPIO_NUM_14;
static const gpio_num_t limit_pin = GPIO_NUM_15;
static const gpio_num_t led_pin = GPIO_NUM_2;

// Potentiometer input pin and values averaged over specified number of readings.
static const int32_t multi_sample_count = 4096;
static const adc_channel_t channel = ADC_CHANNEL_6; // Translates to GPIO_NUM_34 if used with ADC1

// ADC values as a result of using 12-bit width. Do not change unless changing ADC_WIDTH_BIT_12.
static const int32_t adc_min = 0;
static const int32_t adc_max = 4095;

// PWM ranges as a result of timer resolution. Do not change unless changing from LEDC_TIMER_5_BIT.
static const int32_t freq_min = 500; // Hz
static const int32_t freq_max = 160000; // Hz

// Parameters for how potentiometer value is translated to motor velocity.
// Free to tune as appropriate for application. ("Season to taste")
static const int32_t speed_min = freq_min; // Hz. Requires: freq_min <= speed_min < speed_max
static const int32_t speed_max = 149927; // Hz Requires: speed_min < speed_max <= freq_max
static const int32_t update_period = 10; // milliseconds to wait between updates
static const int32_t accel_limit = 5000; // Hz. Speed change per update will not exceed this amount


// Setup LCD
static i2c_dev_t pcf8574;
static esp_err_t write_lcd_data(const hd44780_t *lcd, uint8_t data)
{
    return pcf8574_port_write(&pcf8574, data);
}


bool direction_value = 0;

hd44780_t lcd = {
    .write_cb = write_lcd_data, // use callback to send data to LCD by I2C GPIO expander
        .font = HD44780_FONT_5X8,
        .lines = 2,
        .pins = {
            .rs = 0,
            .e  = 2,
            .d4 = 4,
            .d5 = 5,
            .d6 = 6,
            .d7 = 7,
            .bl = 3
        }
    };

// Init LCD and check for errors
void lcd_test(void *pvParameters)
{


    memset(&pcf8574, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, 0x3F, 0, 21, 22));

    ESP_ERROR_CHECK(hd44780_init(&lcd));

    hd44780_switch_backlight(&lcd, true);
    vTaskDelete(NULL);
}

void addThousandSeparators(char *output, int value) {
    char temp[16];  // Assuming a reasonable length
    sprintf(temp, "%d", value);

    int len = strlen(temp);
    int separatorCount = (len - 1) / 3;

    int newIndex = 0;
    for (int i = 0; i < len; ++i) {
        output[newIndex++] = temp[i];
        if ((len - i - 1) % 3 == 0 && separatorCount > 0) {
            output[newIndex++] = ',';  // Add a comma after every three digits
            --separatorCount;
        }
    }

    output[newIndex++] = ' ';
    output[newIndex++] = ' ';
    output[newIndex++] = ' ';

    output[newIndex] = '\0';  // Null-terminate the string
}

void app_main(void)
{


    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(lcd_test, "lcd_test", 4096, NULL, 5, NULL);
    


    /////////////////////////////////////////////////////////////////////////
    //
    // Parameter checking
    //
    if (adc_max < adc_min) {
        printf("INVALID PARAMETER: ADC max %ld is less than min %ld\n", adc_max, adc_min);
        return;
    }
    if (speed_min < freq_min) {
        printf("INVALID PARAMETER: Specified min speed %ld below PWM minimum %ld\n", speed_min, freq_min);
        return;
    }
    if (speed_min > speed_max) {
        printf("INVALID PARAMETER: Specified min speed %ld exceeds specified top speed %ld\n", speed_min, speed_max);
        return;
    }
    if (speed_max > freq_max) {
        printf("INVALID PARAMETER: Specified top speed %ld exceeds PWM maximum %ld\n", speed_max, freq_max);
        return;
    }


    /////////////////////////////////////////////////////////////////////////
    //
    // Configure timer to be used by LEDC peripheral, then configure LEDC
    // to pulse our STEP pin.
    //
    // Only certain combinations of PWM frequency vs. duty cycle are allowed.
    // The higher the frequency, the coarser the control over duty cycle.
    // At 40MHz only 1-bit duty cycle control is possible: 0% 50%, or 100%.
    // At 20MHz, 2-bit, etc.
    //
    ledc_timer_config_t ledc_timer = {
        // Running Timer 0 in high speed mode. Not picky about which source
        // clock to use, so let it auto-select.
        .timer_num = LEDC_TIMER_0,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .clk_cfg = LEDC_AUTO_CLK,

        .freq_hz = 160000,
        .duty_resolution = LEDC_TIMER_1_BIT,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        // Listening to beat of our drummer, high speed timer 0
        .timer_sel  = LEDC_TIMER_0,
        .speed_mode = LEDC_HIGH_SPEED_MODE,

        // Details for this output
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0, // Start out stopped (0% duty cycle)
        .hpoint     = 0,
        .gpio_num   = step_pin,
    };
    ledc_channel_config(&ledc_channel);

    /////////////////////////////////////////////////////////////////////////
    //
    // Configure digital output for our DIRECTION pin.
    //  No interrupts will be driven by this pin
    //  No internal pull-down or pull-up resistors
    //
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .pin_bit_mask = (1ULL<<direction_pin),
    };
    gpio_config(&io_conf);

   /////////////////////////////////////////////////////////////////////////
    //
    // Configure digital input for our LIMIT pin.
    //  No interrupts will be driven by this pin
    //  Internal pull-up resistor active
    //
    gpio_config_t io_conf_limit = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
        .pull_down_en = 0,
        .pull_up_en = 1,
        .pin_bit_mask = (1ULL<<limit_pin),
    };
    gpio_config(&io_conf_limit);

    /////////////////////////////////////////////////////////////////////////
    //
    // Configure digital output for our LED pin.
    //  No interrupts will be driven by this pin
    //  No internal pull-down or pull-up resistors
    //
    gpio_config_t io_conf_led = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .pin_bit_mask = (1ULL << led_pin),
    };
    gpio_config(&io_conf_led);

    /////////////////////////////////////////////////////////////////////////
    //
    // Configure input pin for analog-to-digital conversion (ADC) to read
    // potentiometer position.
    //  12-bit ADC = results will be between 0 and 4095 inclusive
    //  11 db attenuation allows reading the full range of 3.3V
    //
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);

    /////////////////////////////////////////////////////////////////////////
    //
    //  Variables we'll need for calculation during main loop
    //
    int32_t adc_cumulative; // Accumulated value of multiple ADC samples
    int32_t adc_average;    // Average of multiple ADC samples [adc_min, adc_max]

    int32_t speed_target = 0;   // Speed targeted by ADC
    int32_t speed_current = 0;  // Speed within acceleration limites sent to PWM.

    /////////////////////////////////////////////////////////////////////////
    //
    //  Preparation complete, start main loop
    //
    while (1) {
        // Since ADC is a noisy process, take multiple readings.
        adc_cumulative = 0;
        for (int32_t i = 0; i < multi_sample_count; i++) {
            adc_cumulative += adc1_get_raw(channel);
        }

        // Then take the average of all those values.
        adc_average = adc_cumulative / multi_sample_count;

        
 
        speed_target = (int)((float)adc_average / adc_max * speed_max);

        // Calculate new speed based on target and acceleration limit
        if (speed_target > speed_current + accel_limit) {
            speed_current += accel_limit;
        }
        else if (speed_target < speed_current - accel_limit) {
            speed_current -= accel_limit;
        }
        else {
            speed_current = speed_target;
        }

        // Uncomment next line for diagnostic output
        // printf("Speed target %ld -- current %ld\n", speed_target, speed_current);

        // Display the frequency on the LCD
        char lcd_text[16]; 
        snprintf(lcd_text, sizeof(lcd_text), "Stepper Dir  %s", (direction_value ? "->" : "<-"));
        hd44780_gotoxy(&lcd, 0, 0);
        hd44780_puts(&lcd, lcd_text);
        addThousandSeparators(lcd_text, abs(speed_current));
        hd44780_gotoxy(&lcd, 0, 1);
        hd44780_puts(&lcd, lcd_text);
        snprintf(lcd_text, sizeof(lcd_text), "Hertz");
        hd44780_gotoxy(&lcd, 11, 1);
        hd44780_puts(&lcd, lcd_text);

// Check the state of the limit switch
if (gpio_get_level(limit_pin) == 1) {
    // Limit switch pressed, adjust motor control as needed
    gpio_set_level(led_pin, 1);
    printf("Limit switch activated. LED enabled and motor reversed.\n");
    direction_value = !direction_value;
    gpio_set_level(direction_pin, direction_value);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(led_pin, 0);
} 

        if (abs(speed_current) < speed_min) {
            // Deadband. PWM duty cycle zero. PWM frequency irrelevant.
            ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
            ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
        }
        else {

        gpio_set_level(direction_pin, direction_value);
        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 1);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
        ledc_set_freq(ledc_timer.speed_mode, ledc_timer.timer_num, speed_current);
        }

        // Wait before repeating, yielding to ESP32 housekeeping chores
        vTaskDelay(pdMS_TO_TICKS(update_period));
    }
}