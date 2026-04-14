/**
 * @file as5600.h
 * @brief AS5600 magnetic encoder driver (software I2C)
 */
#ifndef __AS5600_H
#define __AS5600_H

#include "main.h"
#include "gpio.h"
#include "motor/motor_runtime_param.h"

/* ---- Software I2C GPIO macros ---- */

/* Set SCL pin high/low via BSRR/BRR for fast GPIO access */
#define IIC_SCL(x)        do{  \
                              if (x)                                \
                                  SCL_GPIO_Port->BSRR = SCL_Pin;   \
                              else                                  \
                                  SCL_GPIO_Port->BRR  = SCL_Pin;   \
                          }while(0)

/* Set SDA pin high/low via BSRR/BRR for fast GPIO access */
#define IIC_SDA(x)        do{  \
                              if (x)                                \
                                  SDA_GPIO_Port->BSRR = SDA_Pin;   \
                              else                                  \
                                  SDA_GPIO_Port->BRR  = SDA_Pin;   \
                          }while(0)

/* Read SDA pin level from IDR register */
#define IIC_READ_SDA      ((SDA_GPIO_Port->IDR & SDA_Pin) ? 1 : 0)

#define AS5600_I2C_ADDR 0x36    /* AS5600 7-bit I2C address */

/**
 * @brief AS5600 register address enumeration
 */
typedef enum {
    AS5600_ZMCO     = 0x00,     /* ZMCO (burn count) */
    AS5600_ZPOS_H   = 0x01,     /* Zero position high byte */
    AS5600_ZPOS_L   = 0x02,     /* Zero position low byte */
    AS5600_MPOS_H   = 0x03,     /* Max position high byte */
    AS5600_MPOS_L   = 0x04,     /* Max position low byte */
    AS5600_RAM_H    = 0x0C,     /* Raw angle high byte */
    AS5600_RAM_L    = 0x0D,     /* Raw angle low byte */
    AS5600_ANG_H    = 0x0E,     /* Filtered angle high byte */
    AS5600_ANG_L    = 0x0F,     /* Filtered angle low byte */
    AS5600_SRARUS   = 0x0B,     /* Status register */
} AS5600_Reg_t;

/**
 * @brief  Read the current angle from AS5600 (12-bit, 0~4095)
 * @retval 0~4095 on success, 0xFFFF on failure
 */
uint16_t as5600_read_angle(void);

#endif
