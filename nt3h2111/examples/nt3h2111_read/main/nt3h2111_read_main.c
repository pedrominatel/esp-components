/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * NT3H2111 NFC/I2C Bridge Example
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "nt3h2111.h"

static const char *TAG = "NT3H2111-Example";

// I2C Configuration - adjust for your hardware
#define I2C_MASTER_SCL_IO           CONFIG_NT3H2111_I2C_SCL
#define I2C_MASTER_SDA_IO           CONFIG_NT3H2111_I2C_SDA
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          CONFIG_NT3H2111_I2C_CLK_SPEED_HZ
#define FD_GPIO                     CONFIG_NT3H2111_FD_PIN

// Global handle for ISR access
static nt3h2111_handle_t g_nt3h2111 = NULL;
static volatile bool g_field_changed = false;

static void IRAM_ATTR field_detect_isr_handler(void *arg)
{
    // Set flag to handle in main task context
    g_field_changed = true;
}

static void print_block_data(const char* label, uint8_t block_num, const uint8_t *data)
{
    ESP_LOGI(TAG, "%s (Block 0x%02X):", label, block_num);
    printf("  ");
    for (int i = 0; i < NT3H2111_BLOCK_SIZE; i++) {
        printf("%02X ", data[i]);
        if ((i + 1) % 8 == 0 && i < NT3H2111_BLOCK_SIZE - 1) {
            printf("\n  ");
        }
    }
    printf("\n");
}

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "NT3H2111 NFC/I2C Bridge Example");
    ESP_LOGI(TAG, "I2C SDA: GPIO%d, SCL: GPIO%d, Speed: %d Hz", 
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    ESP_LOGI(TAG, "Field Detection: GPIO%d", FD_GPIO);

    // Configure I2C master bus
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return;
    }

    // Create NT3H2111 device
    nt3h2111_handle_t nt3h2111 = nt3h2111_device_create(bus_handle, NT3H2111_I2C_ADDR, I2C_MASTER_FREQ_HZ);
    if (nt3h2111 == NULL) {
        ESP_LOGE(TAG, "Failed to create NT3H2111 device");
        i2c_del_master_bus(bus_handle);
        return;
    }
    g_nt3h2111 = nt3h2111;

    ESP_LOGI(TAG, "NT3H2111 initialized successfully");

    // Initialize field detection GPIO with interrupt
    if (FD_GPIO >= 0) {
        ret = nt3h2111_init_field_detect_gpio(FD_GPIO, field_detect_isr_handler, NULL);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize FD GPIO, will use I2C polling only");
        } else {
            // Read initial FD pin state
            bool field_present = false;
            nt3h2111_read_fd_pin(FD_GPIO, &field_present);
            ESP_LOGI(TAG, "Initial FD pin state: %s (GPIO level: %d)", 
                     field_present ? "FIELD PRESENT" : "NO FIELD",
                     gpio_get_level(FD_GPIO));
        }
    }

    // Read and display session registers
    uint8_t session_regs[NT3H2111_BLOCK_SIZE];
    ret = nt3h2111_read_session_regs(nt3h2111, session_regs);
    if (ret == ESP_OK) {
        print_block_data("Session Registers", NT3H2111_SESSION_REGS, session_regs);
        
        // Check field detect status
        bool field_present = false;
        if (nt3h2111_is_field_present(nt3h2111, &field_present) == ESP_OK) {
            ESP_LOGI(TAG, "NFC Field Status: %s", field_present ? "PRESENT" : "NOT PRESENT");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read session registers: %s", esp_err_to_name(ret));
    }

    // Read and display configuration block
    uint8_t config[NT3H2111_BLOCK_SIZE];
    ret = nt3h2111_read_config(nt3h2111, config);
    if (ret == ESP_OK) {
        print_block_data("Configuration Block", NT3H2111_CONFIG_BLOCK, config);
    } else {
        ESP_LOGE(TAG, "Failed to read configuration: %s", esp_err_to_name(ret));
    }

    // Example: Read first user block
    uint8_t user_data[NT3H2111_BLOCK_SIZE];
    ret = nt3h2111_read_block(nt3h2111, 0x01, user_data);
    if (ret == ESP_OK) {
        print_block_data("User Block 0x01", 0x01, user_data);
    } else {
        ESP_LOGE(TAG, "Failed to read user block: %s", esp_err_to_name(ret));
    }

    // Example: Write data to a user block (block 0x01)
    ESP_LOGI(TAG, "Writing test data to user block 0x01...");
    uint8_t write_data[NT3H2111_BLOCK_SIZE] = {
        'H', 'e', 'l', 'l', 'o', ' ', 'N', 'T',
        '3', 'H', '2', '1', '1', '1', '!', 0x00
    };
    
    ret = nt3h2111_write_block(nt3h2111, 0x01, write_data);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Write successful");
        
        // Wait for EEPROM write to complete (NT3H2111 datasheet: up to 10ms)
        // Add margin for reliability
        ESP_LOGI(TAG, "Waiting for EEPROM write completion...");
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Read back to verify
        uint8_t verify_data[NT3H2111_BLOCK_SIZE];
        ret = nt3h2111_read_block(nt3h2111, 0x01, verify_data);
        if (ret == ESP_OK) {
            print_block_data("Verified User Block 0x01", 0x01, verify_data);
            
            // Compare data
            if (memcmp(write_data, verify_data, NT3H2111_BLOCK_SIZE) == 0) {
                ESP_LOGI(TAG, "Write verification: SUCCESS");
            } else {
                ESP_LOGW(TAG, "Write verification: MISMATCH");
            }
        } else {
            ESP_LOGE(TAG, "Write verification failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "Failed to write user block: %s", esp_err_to_name(ret));
    }

    // Continuous monitoring loop with interrupt-driven field detection
    ESP_LOGI(TAG, "Starting NFC field monitoring (interrupt-driven)...");
    ESP_LOGI(TAG, "Note: I2C access is blocked while RF field is active (normal behavior)");
    
    while (1) {
        if (g_field_changed) {
            g_field_changed = false;
            
            // Read FD pin state (hardware-based, always reliable)
            bool field_present = false;
            if (FD_GPIO >= 0) {
                ret = nt3h2111_read_fd_pin(FD_GPIO, &field_present);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "========================================");
                    ESP_LOGI(TAG, "NFC Field %s (FD pin: %d)", 
                             field_present ? "DETECTED" : "REMOVED",
                             gpio_get_level(FD_GPIO));
                    
                    if (!field_present) {
                        // Field just removed - NOW we can safely read via I2C
                        // Wait a bit for device to be ready
                        vTaskDelay(pdMS_TO_TICKS(50));
                        
                        // Read session registers after field removal
                        ret = nt3h2111_read_session_regs(nt3h2111, session_regs);
                        if (ret == ESP_OK) {
                            print_block_data("Session Registers (Field Removed)", NT3H2111_SESSION_REGS, session_regs);
                            
                            // Check RF_BUSY bit (byte 0, bit 1)
                            bool rf_busy = (session_regs[0] & 0x02) != 0;
                            ESP_LOGI(TAG, "RF Communication was: %s", rf_busy ? "ACTIVE" : "IDLE");
                        } else {
                            ESP_LOGW(TAG, "I2C read failed after field removal");
                        }
                        
                        // Read data that might have been written via NFC
                        uint8_t sram_data[NT3H2111_BLOCK_SIZE];
                        ret = nt3h2111_read_block(nt3h2111, 0xF8, sram_data);
                        if (ret == ESP_OK) {
                            print_block_data("SRAM Block (may contain NFC data)", 0xF8, sram_data);
                        }
                    } else {
                        // Field detected - I2C will be blocked, use GPIO only
                        ESP_LOGI(TAG, "RF field active - I2C access blocked (use GPIO FD pin)");
                        ESP_LOGI(TAG, "NFC reader is communicating with the tag...");
                    }
                    ESP_LOGI(TAG, "========================================");
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Cleanup (won't reach here in this example)
    nt3h2111_device_delete(nt3h2111);
    i2c_del_master_bus(bus_handle);
}
