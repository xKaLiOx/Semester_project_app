

#include <zephyr/kernel.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/dt-bindings/regulator/npm13xx.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/drivers/mfd/npm13xx.h>

int main(void)
{
        printk("Main init started\n");
        const struct device *pmic = DEVICE_DT_GET(DT_NODELABEL(npm1300));

        if (!device_is_ready(pmic))
        {
                printk("nPM1300 not ready\n");
        }
        else
        {
                printk("nPM1300 ready\n");
        }
        return 0;
}
