#pragma once

void vl53l0x_task(void *pvParameters);
void vl53l0x_hard_reset(void);

// vl53l0x se radi na arduinu jer i2c na esp ne radi
// 