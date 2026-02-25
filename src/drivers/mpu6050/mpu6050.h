#ifndef MPU6050_I2C_H
#define MPU6050_I2C_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>
#include <math.h>

#define MPU6050_ADDR 0x68 // Endereço I2C do MPU6050
#define I2C0_SDA_PIN 0
#define I2C0_SCL_PIN 1

void mpu6050_init(void);
void mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az);
float mpu6050_get_inclination(int16_t ax, int16_t ay, int16_t az);

#endif // MPU6050_I2C_H