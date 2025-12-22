

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/gpio.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/drivers/sensor.h>

#include <zephyr/drivers/regulator.h>
#include <zephyr/dt-bindings/regulator/npm13xx.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/drivers/mfd/npm13xx.h>

// DEFINES
#define STACKSIZE 1024
#define PRIORITY 7
// NPM1300
#define UPDATE_TIME_MS 2000
#define PMIC_DEVICE DEVICE_DT_GET(DT_NODELABEL(npm1300))
#define PMIC_LEDS DEVICE_DT_GET(DT_NODELABEL(npm1300_leds))
#define PMIC_GPIO DEVICE_DT_GET(DT_NODELABEL(npm1300_gpio))
#define PMIC_CHARGER DEVICE_DT_GET(DT_NODELABEL(npm1300_charger))
#define PMIC_REGULATORS DEVICE_DT_GET(DT_NODELABEL(npm1300_regulators))

// function declarations
static void event_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void read_sensors(void);
bool configure_events(void);
void pmic_measurements(void);

static const struct device *pmic = PMIC_DEVICE;
static const struct device *leds = PMIC_LEDS;
static const struct device *regulators = PMIC_REGULATORS;
static const struct device *charger = PMIC_CHARGER;
static const struct device *gpio = PMIC_GPIO;

static volatile bool vbus_connected;

// thread for power measurements
#define PMIC_STACK_SIZE 1024
#define PMIC_PRIORITY 7
K_THREAD_STACK_DEFINE(pmic_stack, PMIC_STACK_SIZE);
struct k_thread pmic_thread;

int main(void)
{
	if (!configure_events())
	{
		printk("Error: could not configure events\n");
		return 0;
	}
	printk("PMIC device ok\n");
	// start pmic measurement thread after 1 second
	k_thread_create(&pmic_thread, pmic_stack,
					PMIC_STACK_SIZE,
					pmic_measurements,
					NULL, NULL, NULL,
					PMIC_PRIORITY, 0,
					K_MSEC(1000));
	return 0;//main thread completed
}

void pmic_measurements(void)
{
	while (1)
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

bool configure_events(void)
{
	if (!device_is_ready(pmic))
	{
		printk("Pmic device not ready.\n");
		return false;
	}
	if (!device_is_ready(regulators))
	{
		printk("Regulator device not ready.\n");
		return false;
	}
	if (!device_is_ready(leds))
	{
		printk("Error: led device is not ready\n");
		return false;
	}
	if (!device_is_ready(charger))
	{
		printk("Charger device not ready.\n");
		return false;
	}
	if (!device_is_ready(gpio))
	{
		printk("GPIO device not ready.\n");
		return false;
	}

	// pmic callback
	static struct gpio_callback event_cb;

	gpio_init_callback(&event_cb, event_callback,
					   BIT(NPM13XX_EVENT_VBUS_DETECTED) |
						   BIT(NPM13XX_EVENT_VBUS_REMOVED) |
						   BIT(NPM13XX_EVENT_GPIO1_EDGE) |
						   BIT(NPM13XX_EVENT_GPIO0_EDGE) |
						   BIT(NPM13XX_EVENT_CHG_COMPLETED) |
						   BIT(NPM13XX_EVENT_CHG_ERROR));

	mfd_npm13xx_add_callback(pmic, &event_cb);

	// read initial vbus status
	struct sensor_value val;
	int ret = sensor_attr_get(charger, SENSOR_CHAN_CURRENT, SENSOR_ATTR_UPPER_THRESH, &val);
	if (ret < 0)
	{
		return false;
	}

	vbus_connected = (val.val1 != 0) || (val.val2 != 0);
	printk("Initial Vbus status: %s\n", vbus_connected ? "connected" : "disconnected");
	return true;
}

static void event_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (pins & BIT(NPM13XX_EVENT_VBUS_DETECTED))
	{
		printk("Vbus connected\n");
		vbus_connected = true;
	}
	if (pins & BIT(NPM13XX_EVENT_VBUS_REMOVED))
	{
		printk("Vbus removed\n");
		vbus_connected = false;
	}
	if (pins & BIT(NPM13XX_EVENT_CHG_ERROR))
	{
		printk("Charger error detected!\n");
	}
	if (pins & BIT(NPM13XX_EVENT_GPIO1_EDGE))
	{
		printk("GPIO1 event detected!\n");
	}
	if (pins & BIT(NPM13XX_EVENT_GPIO0_EDGE))
	{
		printk("Reset the pmic!\n");
		mfd_npm13xx_reset(pmic);
	}
	if (pins & BIT(NPM13XX_EVENT_CHG_COMPLETED))
	{
		printk("Charging completed!\n");
	}
}