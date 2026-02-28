

## Migration Guide: v0.2.0 → v0.3.0

**⚠️ Breaking Changes:** Version 0.3.0 introduces handle-based API for thread safety and multi-instance support.

### What Changed

**v0.2.0 (Legacy - Global State):**
```c
// Old API - used global variables
esp_err_t hp203b_init(i2c_master_bus_handle_t bus_handle);
void hp203b_denit(void);
int32_t hp203b_get_press(void);
void hp203b_set_press(uint32_t press_value);
esp_err_t hp203b_read_press(void);
```

**v0.3.0 (Current - Handle-Based):**
```c
// New API - uses device handles
esp_err_t hp203b_init(i2c_master_bus_handle_t bus_handle, hp203b_handle_t *dev_handle);
void hp203b_deinit(hp203b_handle_t dev_handle);  // Note: renamed from denit
int32_t hp203b_get_press(hp203b_handle_t dev_handle);
esp_err_t hp203b_read_press(hp203b_handle_t dev_handle);
// hp203b_set_press() removed - internal state only
```

### How to Migrate

**Step 1: Add device handle variable**
```c
// OLD
i2c_master_bus_handle_t bus_handle;

// NEW - add sensor handle
i2c_master_bus_handle_t bus_handle;
hp203b_handle_t sensor_handle = NULL;  // Add this
```

**Step 2: Update initialization**
```c
// OLD
hp203b_init(bus_handle);

// NEW - pass handle by reference
hp203b_init(bus_handle, &sensor_handle);
```

**Step 3: Pass handle to all function calls**
```c
// OLD
hp203b_read_press();
uint32_t pressure = hp203b_get_press();

// NEW - pass sensor_handle
hp203b_read_press(sensor_handle);
uint32_t pressure = hp203b_get_press(sensor_handle);
```

**Step 4: Update cleanup (if used)**
```c
// OLD
hp203b_denit();

// NEW - cleanup with handle
hp203b_deinit(sensor_handle);
```

**Step 5: Remove hp203b_set_press() calls**
```c
// OLD - manually setting pressure (not recommended)
hp203b_set_press(press_value);

// NEW - removed, pressure is managed internally
// Just use hp203b_read_press() and hp203b_get_press()
```

### Why This Change?

**Thread Safety:** The old implementation used global variables, making it unsafe for:
- Multiple threads accessing the sensor simultaneously
- Multiple HP203B sensors on different I2C addresses
- Concurrent operations in multi-core applications

**Benefits of v0.3.0:**
- ✅ Multiple sensor instances supported
- ✅ Thread-safe operation
- ✅ No global state pollution
- ✅ Cleaner API following ESP-IDF conventions
- ✅ Proper resource cleanup

### Complete Migration Example

**Before (v0.2.0):**
```c
void app_main(void) {
    i2c_master_bus_handle_t bus;
    init_i2c(&bus);
    
    if (hp203b_init(bus) == ESP_OK) {
        while (1) {
            hp203b_read_press();
            ESP_LOGI(TAG, "Pressure: %lu", hp203b_get_press());
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    hp203b_denit();  // Cleanup
}
```

**After (v0.3.0):**
```c
void app_main(void) {
    i2c_master_bus_handle_t bus;
    hp203b_handle_t sensor = NULL;  // Add handle
    init_i2c(&bus);
    
    if (hp203b_init(bus, &sensor) == ESP_OK) {  // Pass &sensor
        while (1) {
            hp203b_read_press(sensor);  // Pass handle
            ESP_LOGI(TAG, "Pressure: %lu", hp203b_get_press(sensor));  // Pass handle
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    hp203b_deinit(sensor);  // Pass handle, note: renamed function
}
```
