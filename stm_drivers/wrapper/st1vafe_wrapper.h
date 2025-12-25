#ifndef ST1VAFE_WRAPPER_H
#define ST1VAFE_WRAPPER_H

#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/device.h>      // for device_is_ready()
#include <zephyr/drivers/spi.h> // for SPI definitions
#include <zephyr/sys/printk.h>  // optional, for debug printing

#include "st1vafe_wrapper.h"
#include "st1vafe6ax_reg.h"
#include "st1vafe3bx_reg.h"

#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB // MSB first, 8 bits per word

extern const struct spi_dt_spec spi20_3bx;
extern const struct spi_dt_spec spi21_6ax;

int32_t zephyr_spi_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t zephyr_spi_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len);
int st1vafe_init(void);

extern stmdev_ctx_t st1vafe3bx_ctx;
extern stmdev_ctx_t st1vafe6ax_ctx;

#endif // ST1VAFE_WRAPPER_H