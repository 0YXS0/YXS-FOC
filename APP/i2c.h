#ifndef I2C_H
#define I2C_H

#include "gd32f30x.h"

#define I2C0_SLAVE_ADDRESS7      0xA0
#define I2C0_SPEED               1000000U
#define I2C0_PAGE_SIZE           8
#define I2C0_GPIO_CLOCK          RCU_GPIOB
#define I2C0_SCL_PORT            GPIOB
#define I2C0_SDA_PORT            GPIOB
#define I2C0_SCL_PIN             GPIO_PIN_6
#define I2C0_SDA_PIN             GPIO_PIN_7

/* configure the I2C interfaces */
void i2c0_config(void);

int8_t i2c_write(uint32_t I2CX, uint8_t DevAddress, uint8_t RegAddress, const uint8_t* data, uint8_t dataSize);
int8_t i2c_read(uint32_t I2CX, uint8_t DevAddress, uint8_t RegAddress, uint8_t* data, uint8_t dataSize);

#endif  /* I2C_H */
