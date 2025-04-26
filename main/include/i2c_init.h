#pragma once

#include "driver/i2c_master.h"

// Define SDA and SCL pins
#define VL53L0X_SDA_GPIO 21
#define VL53L0X_SCL_GPIO 47
#define VL53L0X_XSHUT_GPIO GPIO_NUM_3

extern i2c_master_bus_handle_t vl53l0x_i2c_bus;

void init_i2c_master_vl53l0x(void);
void vl53l0x_hard_reset(void);
