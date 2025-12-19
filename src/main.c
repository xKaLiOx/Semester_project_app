

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

#include <zephyr/drivers/regulator.h>
#include <zephyr/dt-bindings/regulator/npm13xx.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/drivers/mfd/npm13xx.h>

//DEFINES

//NPM1300
#define UPDATE_TIME_MS 2000
#define PMIC_DEVICE DEVICE_DT_GET(DT_NODELABEL(npm1300))
#define PMIC_CHARGER DEVICE_DT_GET(DT_NODELABEL(npm1300_charger))

//function declarations
void read_sensors(void);

static const struct device *pmic = PMIC_DEVICE;
static const struct device *charger = PMIC_CHARGER;

int main(void)
{
        printk("Main init started\n");


        if (!device_is_ready(pmic))
        {
                printk("nPM1300 not ready\n");
        }
        else
        {
                printk("nPM1300 ready\n");
        }
        return 0;

        while(1)
        {
                read_sensors();
                k_msleep(UPDATE_TIME_MS);
        }
}

void read_sensors(void)
{
	struct sensor_value volt;
	struct sensor_value current;
	struct sensor_value error;
	struct sensor_value status;
	struct sensor_value vbus_present;

	sensor_sample_fetch(charger);
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &volt);
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &current);
	sensor_channel_get(charger, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_STATUS,
			   &status);
	sensor_channel_get(charger, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_ERROR, &error);
	sensor_attr_get(charger, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS,
			(enum sensor_attribute)SENSOR_ATTR_NPM13XX_CHARGER_VBUS_PRESENT,
			&vbus_present);

	printk("V: %d.%03d ", volt.val1, volt.val2 / 1000);

	printk("I: %s%d.%04d ", ((current.val1 < 0) || (current.val2 < 0)) ? "-" : "",
	       abs(current.val1), abs(current.val2) / 100);

	printk("Charger Status: %d, Error: %d, VBUS: %s\n", status.val1, error.val1,
	       vbus_present.val1 ? "connected" : "disconnected");
}