/**
 * @file as5600.c
 * @brief AS5600 magnetic encoder driver - software I2C implementation
 */
#include "as5600.h"

/**
 * @brief  Software delay in microseconds (based on NOP loop at 72MHz)
 * @param  us  Delay duration in microseconds
 */
void i2c_delay_us(uint32_t us) {
    uint32_t i;
    for (i = 0; i < 72 * us; i++) {
        __asm("nop");
    }
}

/** @brief Generate I2C START condition: SDA falls while SCL is high */
void i2c_start(void) {
    IIC_SDA(1);
    IIC_SCL(1);
    i2c_delay_us(4);       /* SCL high >= 4us */
    IIC_SDA(0);            /* SDA goes low -> START */
    i2c_delay_us(4);
    IIC_SCL(0);
    i2c_delay_us(4);       /* SCL low >= 4us */
}

/** @brief Generate I2C STOP condition: SDA rises while SCL is high */
void i2c_stop(void) {
    IIC_SCL(0);
    IIC_SDA(0);
    i2c_delay_us(4);
    IIC_SCL(1);
    i2c_delay_us(4);
    IIC_SDA(1);            /* SDA goes high -> STOP */
    i2c_delay_us(4);
}

/**
 * @brief  Wait for ACK from slave device
 * @retval 0 = ACK received, 1 = timeout (NACK)
 */
static uint8_t i2c_wait_ack(void) {
    uint8_t timeout = 0;
    IIC_SDA(1);            /* Release SDA for slave to pull low */
    i2c_delay_us(2);
    IIC_SCL(1);
    i2c_delay_us(2);

    while(IIC_READ_SDA == 1) {
        timeout++;
        if (timeout > 250) {
            i2c_stop();
            return 1;      /* Timeout, no ACK */
        }
    }
    IIC_SCL(0);
    i2c_delay_us(2);
    return 0;               /* ACK received */
}

/**
 * @brief  Write one byte on I2C bus (MSB first), then wait for ACK
 * @param  data  Byte to send
 */
void i2c_write_byte(uint8_t data) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        IIC_SCL(0);
        i2c_delay_us(2);
        if (data & 0x80) {
            IIC_SDA(1);
        } else {
            IIC_SDA(0);
        }
        data <<= 1;
        i2c_delay_us(2);
        IIC_SCL(1);
        i2c_delay_us(4);
    }

    IIC_SCL(0);
    i2c_delay_us(2);
    IIC_SDA(1);            /* Release SDA */
    i2c_delay_us(2);
    IIC_SCL(1);
    i2c_delay_us(4);
    (void)i2c_wait_ack();  /* Wait for slave ACK */
    IIC_SCL(0);
}

/**
 * @brief  Read one byte from I2C bus (MSB first)
 * @param  ack  1 = send ACK after read (more bytes follow), 0 = send NACK (last byte)
 * @retval The byte read from slave
 */
uint8_t i2c_read_byte(uint8_t ack) {
    uint8_t data = 0;
    uint8_t i;
    IIC_SDA(1);            /* Release SDA for slave to drive */
    for (i = 0; i < 8; i++) {
        IIC_SCL(0);
        i2c_delay_us(2);
        IIC_SCL(1);
        i2c_delay_us(4);
        data <<= 1;
        if (IIC_READ_SDA) {
            data |= 1;
        }
    }
    IIC_SCL(0);
    i2c_delay_us(2);
    IIC_SDA(ack ? 0 : 1);  /* ACK=0 (pull low) or NACK=1 (keep high) */
    i2c_delay_us(2);
    IIC_SCL(1);
    i2c_delay_us(4);
    IIC_SCL(0);

    return data;
}

/**
 * @brief  Read the 12-bit angle from AS5600 (register 0x0E~0x0F)
 * @retval 0~4095 on success, 0xFFFF if all retries failed
 */
uint16_t as5600_read_angle(void) {
    uint8_t high_byte, low_byte;
    uint8_t retry = 3;

    while(retry--) {
        /* Send register address (AS5600_ANG_H = 0x0E) */
        i2c_start();
        i2c_write_byte(AS5600_I2C_ADDR << 1);       /* Write mode */
        i2c_write_byte(AS5600_ANG_H);

        /* Repeated start, then read 2 bytes */
        i2c_start();
        i2c_write_byte((AS5600_I2C_ADDR << 1) | 1); /* Read mode */
        high_byte = i2c_read_byte(1);                /* ACK, more data */
        low_byte  = i2c_read_byte(0);                /* NACK, last byte */
        i2c_stop();

        uint16_t angle = (high_byte << 8) | low_byte;
        if (angle <= 4095) {
            return angle;   /* Valid 12-bit angle */
        }
    }
    return 0xFFFF;          /* All retries failed */
}
