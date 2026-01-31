/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * NT3H2111 WiFi Credentials from NFC Example
 * Reads WiFi credentials from NDEF-formatted NFC tag memory
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "nt3h2111.h"

static const char *TAG = "NT3H2111-WiFi";

// I2C Configuration
#define I2C_MASTER_SCL_IO           CONFIG_NT3H2111_I2C_SCL
#define I2C_MASTER_SDA_IO           CONFIG_NT3H2111_I2C_SDA
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          CONFIG_NT3H2111_I2C_CLK_SPEED_HZ
#define FD_GPIO                     CONFIG_NT3H2111_FD_PIN

// WiFi event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static const int MAX_RETRY = 5;

// NDEF and WSC parsing structures
#define NDEF_TNF_WELL_KNOWN    0x01
#define NDEF_TNF_MEDIA         0x02

// WSC Credential Attribute IDs
#define WSC_ID_CREDENTIAL      0x100E
#define WSC_ID_NETWORK_INDEX   0x1026
#define WSC_ID_SSID            0x1045
#define WSC_ID_AUTH_TYPE       0x1003
#define WSC_ID_ENCR_TYPE       0x100F
#define WSC_ID_NETWORK_KEY     0x1027

typedef struct {
    char ssid[33];
    char password[65];
    bool found;
} wifi_credentials_t;

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static uint16_t read_be16(const uint8_t *data)
{
    return (data[0] << 8) | data[1];
}

static bool parse_wsc_credential(const uint8_t *payload, size_t payload_len, wifi_credentials_t *creds)
{
    size_t pos = 0;
    bool ssid_found = false;
    bool password_found = false;
    
    ESP_LOGI(TAG, "Parsing WSC credential, payload length: %d bytes", payload_len);
    
    while (pos + 4 <= payload_len) {
        uint16_t attr_id = read_be16(&payload[pos]);
        uint16_t attr_len = read_be16(&payload[pos + 2]);
        pos += 4;
        
        if (pos + attr_len > payload_len) {
            ESP_LOGW(TAG, "Invalid attribute length at pos %d", pos);
            break;
        }
        
        ESP_LOGD(TAG, "WSC Attribute: 0x%04X, Length: %d", attr_id, attr_len);
        
        switch (attr_id) {
            case WSC_ID_CREDENTIAL:
                ESP_LOGI(TAG, "Found Credential container");
                // Recursively parse credential container
                if (parse_wsc_credential(&payload[pos], attr_len, creds)) {
                    return true;
                }
                break;
                
            case WSC_ID_SSID:
                if (attr_len > 0 && attr_len < sizeof(creds->ssid)) {
                    memcpy(creds->ssid, &payload[pos], attr_len);
                    creds->ssid[attr_len] = '\0';
                    ESP_LOGI(TAG, "SSID: %s", creds->ssid);
                    ssid_found = true;
                }
                break;
                
            case WSC_ID_NETWORK_KEY:
                if (attr_len > 0 && attr_len < sizeof(creds->password)) {
                    memcpy(creds->password, &payload[pos], attr_len);
                    creds->password[attr_len] = '\0';
                    ESP_LOGI(TAG, "Network Key: %s", creds->password);
                    password_found = true;
                }
                break;
                
            case WSC_ID_AUTH_TYPE:
                if (attr_len == 2) {
                    uint16_t auth = read_be16(&payload[pos]);
                    ESP_LOGI(TAG, "Auth Type: 0x%04X", auth);
                }
                break;
                
            case WSC_ID_ENCR_TYPE:
                if (attr_len == 2) {
                    uint16_t encr = read_be16(&payload[pos]);
                    ESP_LOGI(TAG, "Encryption Type: 0x%04X", encr);
                }
                break;
        }
        
        pos += attr_len;
    }
    
    // Mark as found if we have both SSID and password
    if (ssid_found && password_found) {
        creds->found = true;
    }
    
    return creds->found;
}

static bool parse_ndef_message(const uint8_t *data, size_t len, wifi_credentials_t *creds)
{
    size_t pos = 0;
    
    ESP_LOGI(TAG, "Parsing NDEF message, total length: %d bytes", len);
    
    while (pos < len) {
        // Read NDEF record header
        uint8_t header = data[pos++];
        if (header == 0x00 || header == 0xFF) {
            ESP_LOGD(TAG, "Skipping padding byte at pos %d", pos - 1);
            continue;
        }
        
        bool mb = (header & 0x80) != 0;  // Message Begin
        bool me = (header & 0x40) != 0;  // Message End
        bool cf = (header & 0x20) != 0;  // Chunk Flag
        bool sr = (header & 0x10) != 0;  // Short Record
        bool il = (header & 0x08) != 0;  // ID Length present
        uint8_t tnf = header & 0x07;     // Type Name Format
        
        ESP_LOGI(TAG, "NDEF Record: MB=%d ME=%d CF=%d SR=%d IL=%d TNF=0x%02X", 
                 mb, me, cf, sr, il, tnf);
        
        if (pos >= len) break;
        
        uint8_t type_len = data[pos++];
        
        if (pos >= len) break;
        
        uint32_t payload_len;
        if (sr) {
            payload_len = data[pos++];
        } else {
            if (pos + 3 >= len) break;
            payload_len = (data[pos] << 24) | (data[pos+1] << 16) | 
                         (data[pos+2] << 8) | data[pos+3];
            pos += 4;
        }
        
        uint8_t id_len = 0;
        if (il) {
            if (pos >= len) break;
            id_len = data[pos++];
        }
        
        ESP_LOGI(TAG, "Type Length: %d, Payload Length: %d, ID Length: %d", 
                 type_len, payload_len, id_len);
        
        if (pos + type_len + id_len + payload_len > len) {
            ESP_LOGW(TAG, "Invalid NDEF record lengths");
            break;
        }
        
        // Read type
        char record_type[32] = {0};
        if (type_len > 0 && type_len < sizeof(record_type)) {
            memcpy(record_type, &data[pos], type_len);
            record_type[type_len] = '\0';
            ESP_LOGI(TAG, "Record Type: %s", record_type);
        }
        pos += type_len;
        
        // Skip ID if present
        pos += id_len;
        
        // Check if this is a WiFi Simple Configuration record
        if (tnf == NDEF_TNF_MEDIA && type_len == 23 && 
            memcmp(record_type, "application/vnd.wfa.wsc", 23) == 0) {
            
            ESP_LOGI(TAG, "Found WiFi WSC record!");
            if (parse_wsc_credential(&data[pos], payload_len, creds)) {
                return true;
            }
        }
        
        pos += payload_len;
        
        if (me) break;  // Last record in message
    }
    
    return false;
}

static esp_err_t read_wifi_credentials_from_nfc(nt3h2111_handle_t dev_handle, wifi_credentials_t *creds)
{
    memset(creds, 0, sizeof(wifi_credentials_t));
    
    // Read NDEF message from NT3H2111 user memory (starts at block 1)
    // Typically WiFi config fits in first few blocks
    uint8_t ndef_data[16 * 10];  // Read up to 10 blocks (160 bytes)
    size_t total_len = 0;
    
    ESP_LOGI(TAG, "Reading NDEF data from NT3H2111...");
    
    for (uint8_t block = 0x01; block <= 0x0A; block++) {
        esp_err_t ret = nt3h2111_read_block(dev_handle, block, &ndef_data[total_len]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read block 0x%02X", block);
            return ret;
        }
        
        // Print block data
        printf("Block 0x%02X: ", block);
        for (int i = 0; i < 16; i++) {
            printf("%02X ", ndef_data[total_len + i]);
        }
        printf("\n");
        
        total_len += 16;
        
        // Check if we've reached the end (NDEF terminator or padding)
        bool all_zero = true;
        for (int i = 0; i < 16; i++) {
            if (ndef_data[total_len - 16 + i] != 0x00) {
                all_zero = false;
                break;
            }
        }
        if (all_zero && block > 1) break;
    }
    
    // Skip NDEF capability container (first 4 bytes of block 0)
    // For NT3H2111, NDEF message starts at block 1
    // First bytes should be: 0x03 (NDEF Message TLV), then length
    
    size_t ndef_start = 0;
    if (ndef_data[0] == 0x03) {
        // NDEF Message TLV found
        uint8_t len_byte = ndef_data[1];
        if (len_byte == 0xFF) {
            // 3-byte length format
            ndef_start = 4;
        } else {
            // 1-byte length format
            ndef_start = 2;
        }
        
        if (parse_ndef_message(&ndef_data[ndef_start], total_len - ndef_start, creds)) {
            return ESP_OK;
        }
    } else {
        ESP_LOGW(TAG, "NDEF Message TLV not found (first byte: 0x%02X)", ndef_data[0]);
    }
    
    return ESP_FAIL;
}

static esp_err_t wifi_connect_with_credentials(const wifi_credentials_t *creds)
{
    s_wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    strncpy((char *)wifi_config.sta.ssid, creds->ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, creds->password, sizeof(wifi_config.sta.password));
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialization finished.");
    ESP_LOGI(TAG, "Connecting to SSID: %s", creds->ssid);
    
    // Wait for connection or failure
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to SSID: %s", creds->ssid);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s", creds->ssid);
        return ESP_FAIL;
    }
    
    return ESP_FAIL;
}

void app_main(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "NT3H2111 WiFi Credentials Example");
    ESP_LOGI(TAG, "I2C SDA: GPIO%d, SCL: GPIO%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
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
    
    ESP_LOGI(TAG, "NT3H2111 initialized successfully");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Reading WiFi credentials from NFC tag...");
    ESP_LOGI(TAG, "========================================");
    
    // Read WiFi credentials from NFC
    wifi_credentials_t creds;
    ret = read_wifi_credentials_from_nfc(nt3h2111, &creds);
    
    if (ret == ESP_OK && creds.found) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "WiFi Credentials Found!");
        ESP_LOGI(TAG, "SSID: %s", creds.ssid);
        ESP_LOGI(TAG, "Password: %s", creds.password);
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "");
        
        // Connect to WiFi
        ret = wifi_connect_with_credentials(&creds);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Successfully connected to WiFi network!");
        } else {
            ESP_LOGE(TAG, "Failed to connect to WiFi network");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read WiFi credentials from NFC tag");
        ESP_LOGI(TAG, "Make sure the tag contains a WiFi WSC record");
    }
    
    // Keep running
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Cleanup
    nt3h2111_device_delete(nt3h2111);
    i2c_del_master_bus(bus_handle);
}
