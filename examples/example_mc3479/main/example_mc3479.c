#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "mc3479.h"
#include "driver/gpio.h"

#define I2C_MASTER_SDA_IO           7               /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL_IO           8               /*!< gpio number for I2C master clock */
#define I2C_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MASTER_CLK_SPEED        100000           /*!< I2C master clock frequency */

#define ESP_INTR_FLAG_DEFAULT       0
#define GPIO_INTN1                  20              /*!< GPIO pin for interrupt */
#define GPIO_INTN2                  21              /*!< GPIO pin for interrupt */

static const char *TAG = "MC3479 example";
mc3479_handle_t sensor;
static QueueHandle_t gpio_evt_queue = NULL;

// task to read the sensor values
void mc3479_task(void *pvParameters)
{
    int16_t x, y, z = {0};
    while(1)
    {
        //mc3479_get_acceleration(sensor, &x, &y, &z);
        //ESP_LOGI(TAG, "X: %d, Y: %d, Z: %d", x, y, z);
        //mc3479_get_status_reg(sensor, &z);
        //ESP_LOGI(TAG, "Status Register: 0x%02x", z);
        //mc3479_get_interrupt_status_reg(sensor, &z);
        //ESP_LOGI(TAG, "Interrupt Status Register: 0x%02x", z);
        ESP_LOGI(TAG, "Clean all interrupts");
        mc3479_clean_all_interrupts(sensor);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

static void i2c_bus_init(void)
{
    
    esp_err_t err = ESP_OK;
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_CLK_SPEED,
    };

    err = i2c_param_config(I2C_MASTER_NUM, &conf);
    assert(ESP_OK == err);
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus initialization failed");
        return;
    }

}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void intr_task(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

void init_gpio_intr(uint8_t gpio){
    
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL<<gpio);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

}

void create_intr_routine(void)
{
    init_gpio_intr(GPIO_INTN1);
    init_gpio_intr(GPIO_INTN2);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(intr_task, "gpio_task_example", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INTN1, gpio_isr_handler, (void*) GPIO_INTN1);
    gpio_isr_handler_add(GPIO_INTN2, gpio_isr_handler, (void*) GPIO_INTN2);

}

static void mc3479_sensor_init(void)
{
    i2c_bus_init();
    sensor = mc3479_create(I2C_MASTER_NUM, MC3479_I2C_ADDR_0);
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Sensor initialization failed");
        return;
    }
    ESP_LOGI(TAG, "Sensor initialization success");
}

static void mc3479_sensor_denit(void)
{
    mc3479_delete(sensor);
}

static void mc3479_sensor_motion(void)
{
    esp_err_t err = ESP_OK;
    uint8_t motion_intr;
    uint8_t motion;
    
    mc3479_motion_intr_t motion_intr_config = {
        .TILT_INT = false,
        .FLIP_INT = false,
        .ANYM_INT = true,
        .SHAKE_INT = false,
        .TILT_35_INT = false,
        .RESERVED = false,
        .AUTO_CLR = false,
        .ACQ_INT = false
    };

    err = mc3479_set_motion_intr(sensor, motion_intr_config);
    assert(ESP_OK == err);

    mc3479_get_motion_intr(sensor, &motion_intr);
    ESP_LOGI(TAG, "Motion interrupt: 0x%02x", motion_intr);

    mc3479_motion_t motion_config = {
        .TF = false,
        .MOTION_LATCH = true,
        .ANY_MOTION = true,
        .SHAKE = false,
        .TILT_35 = false,
        .Z_AXIS_ORT = false,
        .RAW_PROC_STAT = false,
        .MOTION_RESET = false
    };

    err = mc3479_set_motion(sensor, motion_config);
    assert(ESP_OK == err);
    
    mc3479_get_motion(sensor, &motion);
    ESP_LOGI(TAG, "Motion Config: 0x%02x", motion);

    mc3479_gpio_intr_t gpio_intr_config = {
        .GPIO1_INTN1_IAH = true,
        .GPIO1_INTN1_IPP = true,
        .GPIO2_INTN2_IAH = true,
        .GPIO2_INTN2_IPP = true
    };

    ESP_LOGI(TAG, "Motion Interrupt Set");
    mc3479_set_gpio_intr(sensor, gpio_intr_config);

}

static void mc3479_sensor_start(void)
{
    
    mc3479_config_t cfg;

    mc3479_set_mode(sensor, MC3479_MODE_SLEEP);
    mc3479_clean_all_interrupts(sensor);
    mc3479_sensor_motion();

    // Set and check the range
    mc3479_set_range(sensor, MC3479_RANGE_2G);
    mc3479_get_range(sensor, &cfg.range);
    ESP_LOGI(TAG, "Range: %d", cfg.range);
    // Set and check the sample rate
    mc3479_set_sample_rate(sensor, MC3479_SAMPLE_1000Hz);
    mc3479_get_sample_rate(sensor, &cfg.sample_rate);
    ESP_LOGI(TAG, "Sample rate: %d", cfg.sample_rate);

    // Get the mode
    mc3479_get_mode(sensor, &cfg.mode);
    // Print the mode
    switch (cfg.mode)
    {
    case MC3479_MODE_SLEEP:
        ESP_LOGI(TAG, "Mode: Sleep");
        break;
    case MC3479_MODE_CWAKE:
        ESP_LOGI(TAG, "Mode: Continuous Wake");
        break;
    case MC3479_MODE_RESERVED:
        ESP_LOGI(TAG, "Mode: Reserved");
        break;
    case MC3479_MODE_STANDBY:
        ESP_LOGI(TAG, "Mode: Standby");
        break;    
    default:
        ESP_LOGE(TAG, "Mode: Unknown");
        break;
    }

    if(cfg.mode == MC3479_MODE_SLEEP)
    {
        mc3479_set_mode(sensor, MC3479_MODE_CWAKE);
    }

    // Create a task to read the sensor values
    xTaskCreatePinnedToCore(&mc3479_task, "mc3479_task", 2048, NULL, 5, NULL, 1);

}

void app_main(void)
{
    esp_err_t err = ESP_OK;
    uint8_t chip_id;
    
    create_intr_routine();
    mc3479_sensor_init();
    err = mc3479_get_chip_id(sensor, &chip_id);

    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get chip id");
        return;
    }

    ESP_LOGI(TAG, "Chip ID: 0x%02x", chip_id);
    // Compare the chip if to the default value
    if(chip_id == MC3479_DEFAULT_CHIP_ID)
    {
        ESP_LOGI(TAG, "Chip ID is correct");
        // If the sonsor is connected, start the sensor
        mc3479_sensor_start();
    }
    else
    {
        ESP_LOGE(TAG, "Chip ID is incorrect");
    }

}
