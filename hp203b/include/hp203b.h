#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "driver/i2c_master.h"

esp_err_t hp203b_init(i2c_master_bus_handle_t bus_handle);
void hp203b_denit(void);
int32_t hp203b_get_press(void);
void hp203b_set_press(uint32_t press_value);
esp_err_t hp203b_read_press(void);
esp_err_t hp203b_test(void);

#define HP203B_DEFAULT_ADDRESS              0x76     // (0xEC)>>1
#define HP203B_CONVERSIONDELAY              100
#define HP203B_CMD_SOFT_RESET               0x06
#define HP203B_CMD_CONVERT                  0x40      // Convert the Sensor Output to the Digital Values
#define HP203B_CMD_OSR_4096                 0x00      // Convert D1 (OSR=4096)
#define HP203B_CMD_OSR_2048                 0x04      // OSR: 2048
#define HP203B_CMD_OSR_1024                 0x08      // OSR: 1024
#define HP203B_CMD_OSR_512                  0x0C      // OSR: 512
#define HP203B_CMD_OSR_256                  0x10      // OSR: 256
#define HP203B_CMD_OSR_128                  0x14      // OSR: 128
#define HP203B_CMD_CHNL_PRESTEMP            0x00      // Sensor Pressure and Temperature Channel
#define HP203B_CMD_CHNL_TEMP                0x10      // Sensor Temperature Channel
#define HP203B_CMD_READ_PT                  0x10      // Read Temperature and Pressure Values
#define HP203B_CMD_READ_AT                  0x11      // Read Temperature and Altitude Values
#define HP203B_CMD_READ_P                   0x30      // Read Pressure Value Only
#define HP203B_CMD_READ_A                   0x31      // Read Altitude Value Only
#define HP203B_CMD_READ_T                   0x32      // Read Temperature Value Only
#define HP203B_CMD_ANA_CAL                  0x28      // Re-Calibrate the Internal Analog Blocks
#define HP203B_CMD_READ_REG                 0x80      // Read Out the Control Registers
#define HP203B_CMD_WRITE_REG                0xC0      // Write Out the Control Registers
#define HP203B_CMD_INT_SRC                  0x0D      // Write Out the Control Registers


#ifdef __cplusplus
}
#endif