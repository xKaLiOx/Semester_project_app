#include "st1vafe_wrapper.h"

const struct spi_dt_spec spi20_3bx = SPI_DT_SPEC_GET(DT_NODELABEL(st1vafe3bx), SPIOP, 0);
const struct spi_dt_spec spi21_6ax = SPI_DT_SPEC_GET(DT_NODELABEL(st1vafe6ax), SPIOP, 0);

stmdev_ctx_t st1vafe3bx_ctx = {
    .read_reg = zephyr_spi_read,
    .write_reg = zephyr_spi_write,
    .handle = (void *)(const void *)&spi20_3bx,
};

stmdev_ctx_t st1vafe6ax_ctx = {
    .read_reg = zephyr_spi_read,
    .write_reg = zephyr_spi_write,
    .handle = (void *)(const void *)&spi21_6ax,
};

 int32_t zephyr_spi_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    const struct spi_dt_spec *spi = (const struct spi_dt_spec *)handle;
    int ret;

    uint8_t tx_buf[1] = { 0x80 | reg }; // MSB=1 for read
    struct spi_buf txb[] = {
        { .buf = tx_buf, .len = 1 },
        { .buf = data, .len = len }
    };
    struct spi_buf_set tx = { .buffers = txb, .count = 2 };

    struct spi_buf rxb[] = {
        { .buf = NULL, .len = 1 }, // discard reg echo
        { .buf = data, .len = len }
    };
    struct spi_buf_set rx = { .buffers = rxb, .count = 2 };

    ret = spi_transceive_dt(spi, &tx, &rx);
    return ret < 0 ? ret : 0;
}

 int32_t zephyr_spi_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    const struct spi_dt_spec *spi = (const struct spi_dt_spec *)handle;
    int ret;

    uint8_t tx_buf[1] = { reg & 0x7F }; // MSB=0 for write
    struct spi_buf txb[] = {
        { .buf = tx_buf, .len = 1 },
        { .buf = (uint8_t *)data, .len = len }
    };
    struct spi_buf_set tx = { .buffers = txb, .count = 2 };
    
    ret = spi_write_dt(spi, &tx);
    return ret < 0 ? ret : 0;
}

int st1vafe_init(void)
{   
    if (!spi_is_ready_dt(&spi20_3bx)) {
        printk("ST1VAFE3BX SPI device not ready!\n");
        return -ENODEV;
    }

    if (!spi_is_ready_dt(&spi21_6ax)) {
        printk("ST1VAFE6AX SPI device not ready!\n");
        return -ENODEV;
    }
    return 0;
}