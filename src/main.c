#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/nus.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/dt-bindings/regulator/npm13xx.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/mfd/npm13xx.h>

// DEFINES
// #define PMIC_OFF
#define SENSORS_OFF
// #define DEBUG_LOOP

// // include stm generic drivers
#ifndef SENSORS_OFF
#include "st1vafe_wrapper.h"
#endif

// // BLUETOOTH
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define STACKSIZE 1024
#define PRIORITY 7

// // NPM1300
#define UPDATE_TIME_MS 2000
#define PMIC_DEVICE DEVICE_DT_GET(DT_NODELABEL(npm1300))
#define PMIC_LEDS DEVICE_DT_GET(DT_NODELABEL(npm1300_leds))
#define PMIC_GPIO DEVICE_DT_GET(DT_NODELABEL(npm1300_gpio))
#define PMIC_CHARGER DEVICE_DT_GET(DT_NODELABEL(npm1300_charger))
#define PMIC_REGULATORS DEVICE_DT_GET(DT_NODELABEL(npm1300_regulators))
#define PMIC_REGULATOR_BUCK1 DEVICE_DT_GET(DT_NODELABEL(npm1300_buck1))
#define PMIC_REGULATOR_BUCK2 DEVICE_DT_GET(DT_NODELABEL(npm1300_buck2))

// function declarations
static void pmic_event_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void read_sensors(void);
bool configure_events(void);
void pmic_measurements(void *arg1, void *arg2, void *arg3);

void bt_nus_handler(void *arg1, void *arg2, void *arg3);
static void adv_work_handler(struct k_work *work);
static void advertising_start(void);
static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static void recycled_cb(void);
int send_to_bt(const uint8_t *data, uint16_t len);
static void auth_cancel(struct bt_conn *conn);
static void security_changed(struct bt_conn *conn, bt_security_t level,
							 enum bt_security_err err);
static void send_enabled_cb(enum bt_nus_send_status status);
static void nus_receive_cb(struct bt_conn *conn,
						   const uint8_t *const data, uint16_t len);
static ssize_t bt_read_battery_values(struct bt_conn *conn,
									  const struct bt_gatt_attr *attr,
									  void *buf, uint16_t len,
									  uint16_t offset);
static ssize_t bt_read_hr_rp_values(struct bt_conn *conn,
									const struct bt_gatt_attr *attr,
									void *buf, uint16_t len,
									uint16_t offset);
#ifndef SENSORS_OFF
static bool configure_interrupts(void);
static void st1vafe3bx_interrupt_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void st1vafe6ax_interrupt_cb1(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void st1vafe6ax_interrupt_cb2(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
#endif

static struct bt_nus_cb nus_cb = {
	.received = nus_receive_cb,
	.sent = NULL,
	.send_enabled = send_enabled_cb,
};

typedef struct pmic_data_struct
{
	struct sensor_value voltage;
	struct sensor_value current;
	struct sensor_value charger_status;
	struct sensor_value charger_error;
	struct sensor_value vbus_present;
} pmic_data;

struct sensor_value charger_value;

static const struct device *pmic = PMIC_DEVICE;
static const struct device *leds = PMIC_LEDS;

static const struct device *regulator_buck1 = PMIC_REGULATOR_BUCK1;
static const struct device *regulator_buck2 = PMIC_REGULATOR_BUCK2;
static const struct device *charger = PMIC_CHARGER;
static const struct device *gpio = PMIC_GPIO;

struct gpio_callback st1vafe3bx_interrupt;
struct gpio_callback st1vafe6ax_interrupt1;
struct gpio_callback st1vafe6ax_interrupt2;

#ifndef SENSORS_OFF
static const struct gpio_dt_spec st1vafe3bx_int_specs =
	GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(st1vafe3bx), irq_gpios, 0);
static const struct gpio_dt_spec st1vafe6ax_int_specs1 =
	GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(st1vafe6ax), irq_gpios, 0);
static const struct gpio_dt_spec st1vafe6ax_int_specs2 =
	GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(st1vafe6ax), irq_gpios, 1);
#endif

static volatile bool vbus_connected;

static volatile bool START_MEASUREMENTS = false;
static volatile bool RECEIVED_INTERRUPT = false;
static volatile bool PMIC_MEASUREMENTS_DONE = false;

pmic_data pmic_measurement_data;

// threads
#define PMIC_STACK_SIZE 1024
#define PMIC_PRIORITY 8
#define BT_NUS_PRIORITY 6
#define BT_NUS_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(pmic_stack, PMIC_STACK_SIZE);
K_THREAD_STACK_DEFINE(bt_nus_stack, BT_NUS_STACK_SIZE);
struct k_thread pmic_thread;
struct k_thread bt_nus_thread;

// bluetooth

// GATT defines
#define BT_UUID_BAT_CUSTOM_SERV_VAL BT_UUID_128_ENCODE(0xd618e70c, 0x6cd4, 0x4fb3, 0xb085, 0x47139f2e3990)		  // custom UUID for battery
#define BT_UUID_BAT_CUSTOM_SERV_VAL_INSIDE BT_UUID_128_ENCODE(0xd618e70c, 0x6cd4, 0x4fb3, 0xb085, 0x47139f2e3991) // custom UUID values of battery

#define BT_UUID_HR_RP_CUSTOM_SERV_VAL BT_UUID_128_ENCODE(0x8b9895c7, 0x9570, 0x41d1, 0x8383, 0xb039202bc131)		// custom UUID for heart rate and respiratory rate
#define BT_UUID_HR_RP_CUSTOM_SERV_VAL_INSIDE BT_UUID_128_ENCODE(0x8b9895c7, 0x9570, 0x41d1, 0x8383, 0xb039202bc132) // custom UUID for heart rate and respiratory rate

#define BT_UUID_BAT_CUSTOM_SERVICE BT_UUID_DECLARE_128(BT_UUID_BAT_CUSTOM_SERV_VAL)
#define BT_UUID_BAT_CUSTOM_SERVICE_VAL_INSIDE BT_UUID_DECLARE_128(BT_UUID_BAT_CUSTOM_SERV_VAL_INSIDE)

#define BT_UUID_HR_RP_CUSTOM_SERVICE BT_UUID_DECLARE_128(BT_UUID_HR_RP_CUSTOM_SERV_VAL)
#define BT_UUID_HR_RP_CUSTOM_SERVICE_VAL_INSIDE BT_UUID_DECLARE_128(BT_UUID_HR_RP_CUSTOM_SERV_VAL_INSIDE)

BT_GATT_SERVICE_DEFINE(battery_service,
					   BT_GATT_PRIMARY_SERVICE(BT_UUID_BAT_CUSTOM_SERVICE),
					   BT_GATT_CHARACTERISTIC(BT_UUID_BAT_CUSTOM_SERVICE_VAL_INSIDE,
											  BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
											  bt_read_battery_values, NULL, &pmic_measurement_data.voltage),
					   BT_GATT_CHARACTERISTIC(BT_UUID_BAT_CUSTOM_SERVICE_VAL_INSIDE,
											  BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
											  bt_read_battery_values, NULL, &pmic_measurement_data.current),
					   BT_GATT_CHARACTERISTIC(BT_UUID_BAT_CUSTOM_SERVICE_VAL_INSIDE,
											  BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
											  bt_read_battery_values, NULL, &pmic_measurement_data.charger_status),
					   BT_GATT_CHARACTERISTIC(BT_UUID_BAT_CUSTOM_SERVICE_VAL_INSIDE,
											  BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
											  bt_read_battery_values, NULL, &pmic_measurement_data.charger_error),
					   BT_GATT_CHARACTERISTIC(BT_UUID_BAT_CUSTOM_SERVICE_VAL_INSIDE,
											  BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
											  bt_read_battery_values, NULL, &pmic_measurement_data.vbus_present));

BT_GATT_SERVICE_DEFINE(hr_rp_service,
					   BT_GATT_PRIMARY_SERVICE(BT_UUID_HR_RP_CUSTOM_SERVICE),
					   BT_GATT_CHARACTERISTIC(BT_UUID_HR_RP_CUSTOM_SERVICE_VAL_INSIDE,
											  BT_GATT_CHRC_READ,
											  BT_GATT_PERM_READ,
											  bt_read_hr_rp_values, NULL, NULL));

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

// BT
static struct bt_conn *current_conn;
static bool nus_tx_connected = false;
static struct k_work adv_work;

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled_cb,
	.security_changed = security_changed,
};

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = NULL,
	.passkey_confirm = NULL,
	.cancel = auth_cancel,
	.pairing_confirm = NULL,
};
static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = NULL,
	.pairing_failed = NULL};

static K_SEM_DEFINE(SEM_PMIC_MEASURE, 0, 1);
static K_SEM_DEFINE(SEM_BT_NUS_SEND, 0, 1);

K_MUTEX_DEFINE(MUTEX_RECEIVED_INTERRUPT);
K_MUTEX_DEFINE(MUTEX_DATA_UPDATE);

// // DEBUG GPIOS
static const struct gpio_dt_spec pin2 = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pin2_gpios);
static const struct gpio_dt_spec pin3 = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pin3_gpios);

int main(void)
{
	int ret;
	// FOR DEBUGGING PURPOSES GPIOS

	if (!gpio_is_ready_dt(&pin2))
	{
		printk("Error: pin2 GPIO device not ready\n");
		return 0;
	}

	if (!gpio_is_ready_dt(&pin3))
	{
		printk("Error: pin3 GPIO device not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&pin2, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		printk("Error configuring pin2: %d\n", ret);
		return 0;
	}

	ret = gpio_pin_configure_dt(&pin3, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		printk("Error configuring pin3: %d\n", ret);
		return 0;
	}

	printk("GPIO debug pins configured successfully\n");

#ifdef DEBUG_LOOP
	int counter = 0;
	while (1)
	{
		gpio_pin_toggle_dt(&pin2);

		if (counter % 2 == 0)
		{
			gpio_pin_toggle_dt(&pin3);
		}

		printk("Counter: %d\n", counter++);

		k_sleep(K_MSEC(250)); // 500ms delay = ~1Hz toggle for pin2
	}
#endif

#ifndef PMIC_OFF
	if (!configure_events())
	{
		printk("Error: could not configure events\n");
		return 0;
	}
	printk("PMIC device ok\n");
#endif

#ifndef SENSORS_OFF

	// apeit be funkciju ir wrapperio
	if (!st1vafe_init())
	{
		printk("ST1VAFE sensors intialized\n");
	}

	if (!configure_interrupts()) // 3bx pertrauktis atjungta
	{
		printk("Error: could not configure sensor interrupts\n");
		return 0;
	}

#define SPIOPS (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA)
	const struct spi_dt_spec spi21_6ax_main = SPI_DT_SPEC_GET(DT_NODELABEL(st1vafe6ax), SPIOPS, 0);
	const struct spi_dt_spec spi21_3bx_main = SPI_DT_SPEC_GET(DT_NODELABEL(st1vafe3bx), SPIOPS, 0);

	if (!spi_is_ready_dt(&spi21_3bx_main))
	{
		printk("ST1VAFE3BX SPI device not ready!\n");
		return -ENODEV;
	}
	/////////////////
	uint8_t tx_buf_data[2] = {ST1VAFE3BX_WHO_AM_I | 0x80, 0x00};
	uint8_t rx_buf_data[2];

	struct spi_buf tx_buf = {
		.buf = tx_buf_data,
		.len = 2,
	};

	struct spi_buf rx_buf = {
		.buf = rx_buf_data,
		.len = 2,
	};

	struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	while (1)
	{
		k_msleep(1000);
		printk("Attempting transfer 3BX...\n");
		ret = spi_transceive_dt(&spi21_3bx_main, &tx, &rx);

		if (ret != 0)
		{
			printk("SPI Error: %d\n", ret);
		}
		else
		{
			printk("WHO_AM_I: 0x%02X\n", rx_buf_data[1]); // 1 registras siuksle
		}
	}

	if (!spi_is_ready_dt(&spi21_6ax_main))
	{
		printk("ST1VAFE6AX SPI device not ready!\n");
		return -ENODEV;
	}
	/////////////////
	uint8_t tx_buf_data[2] = {ST1VAFE6AX_WHO_AM_I | 0x80, 0x00};
	uint8_t rx_buf_data[2];

	struct spi_buf tx_buf = {
		.buf = tx_buf_data,
		.len = 2,
	};

	struct spi_buf rx_buf = {
		.buf = rx_buf_data,
		.len = 2,
	};

	struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	while (1)
	{
		k_msleep(1000);
		printk("Attempting transfer...\n");
		ret = spi_transceive_dt(&spi21_6ax_main, &tx, &rx);

		if (ret != 0)
		{
			printk("SPI Error: %d\n", ret);
		}
		else
		{
			printk("WHO_AM_I: 0x%02X\n", rx_buf_data[1]); // 1 registras siuksle
		}
	}

	///////////////

	if (!st1vafe6ax_device_id_get(&st1vafe6ax_ctx, &dummy))
	{
		printk("ST1VAFE6AX detected successfully\n");
	}
	else
	{
		printk("ST1VAFE6AX not detected\n");
	}
	printk("ST1VAFE6AX WHO_AM_I: %c\n", dummy);
	if (dummy == 0x71)
	{
		printk("ST1VAFE6AX WHO_AM_I correct\n");
	}
	if (!st1vafe3bx_device_id_get(&st1vafe3bx_ctx, &dummy))
	{
		printk("ST1VAFE3BX detected successfully\n");
	}
	else
	{
		printk("ST1VAFE3BX not detected\n");
		return 0;
	}
	printk("ST1VAFE3BX WHO_AM_I: %c\n", dummy);
	if (dummy == 0x48)
	{
		printk("ST1VAFE3BX WHO_AM_I correct\n");
	}
#endif

	// start pmic measurement thread after 1 second
	k_thread_create(&pmic_thread, pmic_stack,
					PMIC_STACK_SIZE, pmic_measurements,
					NULL, NULL, NULL,
					PMIC_PRIORITY, 0, K_MSEC(500));

	int err;
	err = bt_enable(NULL);
	if (err)
	{
		printk("BLE enable failed (err: %d)", err);
		return 0;
	}

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err != 0)
	{
		printk("Authentication cb register error (err %d)\n", err);
		return -1;
	}
	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err != 0)
	{
		printk("Authentication cb register info error (err %d)\n", err);
		return -1;
	}

	// err = bt_gatt_exchange_mtu(current_conn, NULL);
	// if (err)
	// {
	// 	printk("MTU exchange failed: %d\n", err);
	// }
	// else
	// {
	// 	printk("MTU exchange requested\n");
	// 	bt_gatt_get_mtu(current_conn);
	// }

	// communication between phone
	err = bt_nus_init(&nus_cb);
	if (err)
	{
		printk("Failed to initialize BT NUS shell (err: %d)\n", err);
		return 0;
	}

	// Enable Bluetooth
	k_thread_create(&bt_nus_thread, bt_nus_stack,
					BT_NUS_STACK_SIZE,
					bt_nus_handler,
					NULL, NULL, NULL,
					BT_NUS_PRIORITY, 0,
					K_MSEC(1000));

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	return 0; // main thread completed
}

void bt_nus_handler(void *arg1, void *arg2, void *arg3)
{
	printk("BT NUS handler started...\n");
	int8_t err = 0;
	pmic_data local_copy; // Local buffer
	bool should_send = false;
	bool send_bt_pmic_data = false;
	static bool last_conn = false;
	while (1)
	{
		if (current_conn)
		{
			if (!last_conn)
			{
				last_conn = true;
			}
			if (k_sem_take(&SEM_BT_NUS_SEND, K_FOREVER) == 0)
			{
				printk("BT NUS handler to SEND semaphore acquired...\n");
				// 	// k_mutex_lock(&MUTEX_RECEIVED_INTERRUPT, K_FOREVER);
				// 	// if (RECEIVED_INTERRUPT)
				// 	// {
				// 	// 	printk("BT NUS handler processing interrupt...\n");
				// 	// 	RECEIVED_INTERRUPT = false;
				// 	// }
				// 	// k_mutex_unlock(&MUTEX_RECEIVED_INTERRUPT);

				k_mutex_lock(&MUTEX_DATA_UPDATE, K_FOREVER);
				printk("update mutex locked\n");
				// should_send = (PMIC_MEASUREMENTS_DONE && START_MEASUREMENTS); only for sensor data, not pmic

				if (PMIC_MEASUREMENTS_DONE)
				{
					printk("data should be sent\n");
					memcpy(&local_copy, &pmic_measurement_data, sizeof(local_copy));
					PMIC_MEASUREMENTS_DONE = false;
					send_bt_pmic_data = true;
				}
				else
				{
					send_bt_pmic_data = false;
				}
				k_sem_give(&SEM_PMIC_MEASURE); // allow new measurement
				k_mutex_unlock(&MUTEX_DATA_UPDATE);
				printk("PMIC MEASURE semaphore given\n");
				printk("update mutex unlocked\n");

				if (send_bt_pmic_data && nus_tx_connected)
				{
					// send over bluetooth (mutex protected)
					uint8_t buffer[100];
					// int len = snprintf((char *)buffer, sizeof(buffer),
					// 				   "V:%d.%03d I:%s%d.%03d CHG_STAT:%d ERR:%d VBUS:%s\n",
					// 				   local_copy.voltage.val1, local_copy.voltage.val2 / 1000,
					// 				   ((local_copy.current.val1 < 0) || (local_copy.current.val2 < 0)) ? "-" : "",
					// 				   abs(local_copy.current.val1), abs(local_copy.current.val2) / 100,
					// 				   local_copy.charger_status.val1,
					// 				   local_copy.charger_error.val1,
					// 				   local_copy.vbus_present.val1 ? "connected" : "disconnected");
					int len = snprintf((char *)buffer, sizeof(buffer),
									   "V:%d.%03d I:%s%d.%03d\n",
									   local_copy.voltage.val1, local_copy.voltage.val2 / 1000,
									   ((local_copy.current.val1 < 0) || (local_copy.current.val2 < 0)) ? "-" : "",
									   abs(local_copy.current.val1), abs(local_copy.current.val2) / 100);
					if (nus_tx_connected)
					{
						if (send_to_bt(buffer, len) < 0)
						{
							printk("Send failed: %d\n", err);
						}
						else
						{
							printk("Sent message to BT NUS\n");
						}
					}
				}
				else
				{
					printk("No data to send over BT NUS (measurements not ready or not started)\n");
				}
			}
		}
		else
		{
			if (last_conn) // only print when state changes
			{
				printk("No connection to BT\n");
			}
			last_conn = false;
			k_msleep(1000);
		}
	}
}

void pmic_measurements(void *arg1, void *arg2, void *arg3)
{
	printk("PMIC thread enabled...\n");
	// Block until first client enables notifications
	static uint8_t err = 0;
	led_off(leds, 1);
	while (1)
	{
		if (current_conn) // read even when START_MEASUREMENTS is off
		{
			if (k_sem_take(&SEM_PMIC_MEASURE, K_NO_WAIT) == 0)
			{
				printk("PMIC measurement semaphore acquired...\n");
				read_sensors();
				if (err)
				{
					printk("Send failed: %d\n", err);
				}
				led_on(leds, 1);
				k_msleep(100);
				led_off(leds, 1);

				if(nus_tx_connected)
				{
					k_sem_give(&SEM_BT_NUS_SEND); // give signal to send from bt nus thread
				}
					
			}
		}
		k_msleep(UPDATE_TIME_MS);
	}
}

void read_sensors(void)
{
	static struct sensor_value volt;
	static struct sensor_value current;
	static struct sensor_value error;
	static struct sensor_value status;
	static struct sensor_value vbus_present;

	sensor_sample_fetch(charger);
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &volt);
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &current);
	sensor_channel_get(charger, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_STATUS,
					   &status);
	sensor_channel_get(charger, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_ERROR, &error);
	sensor_attr_get(charger, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS,
					(enum sensor_attribute)SENSOR_ATTR_NPM13XX_CHARGER_VBUS_PRESENT,
					&vbus_present);

	if (k_mutex_lock(&MUTEX_DATA_UPDATE, K_NO_WAIT) == 0)
	{
		pmic_measurement_data.voltage = volt;
		pmic_measurement_data.current = current;
		pmic_measurement_data.charger_status = status;
		pmic_measurement_data.charger_error = error;
		pmic_measurement_data.vbus_present = vbus_present;
		PMIC_MEASUREMENTS_DONE = true;
		k_mutex_unlock(&MUTEX_DATA_UPDATE);
	}

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
	int32_t volt_uv;
	if (device_is_ready(regulator_buck1))
	{
		regulator_common_data_init(regulator_buck1);
		if (!regulator_get_voltage(regulator_buck1, &volt_uv))
		{
			printk("Regulator buck1 set at %u mV.\n", volt_uv / 1000);
			if (volt_uv < 3200000)
			{
				int ret = regulator_set_voltage(regulator_buck1, 3200000, 3300000);
				if (ret)
				{
					printk("Failed to enable buck1 to 3.3V: %d\n", ret);
				}
				if (!regulator_enable(regulator_buck1))
				{
					printk("Regulator buck1 enabled and set to 3.2-3.3V.\n");
				}
			}
		}
	}
	else
	{
		printk("Regulator 2 device not ready.\n");
	}

	if (device_is_ready(regulator_buck2))
	{
		regulator_common_data_init(regulator_buck2);
		if (!regulator_get_voltage(regulator_buck2, &volt_uv))
		{
			printk("Regulator buck2 set at %u mV.\n", volt_uv / 1000);
			if (volt_uv < 3200000)
			{
				int ret = regulator_set_voltage(regulator_buck2, 3200000, 3300000);
				if (ret)
				{
					printk("Failed to enable buck2 to 3.2-3.3V: %d\n", ret);
				}
				if (!regulator_enable(regulator_buck2))
				{
					printk("Regulator buck2 enabled and set to 3.2-3.3V.\n");
				}
			}
		}
	}
	else
	{
		printk("Regulator 2 device not ready.\n");
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
	if (!device_is_ready(regulator_buck1))
	{
		printk("Regulator buck1 device not ready.\n");
		return false;
	}
	if (!device_is_ready(regulator_buck2))
	{
		printk("Regulator buck2 device not ready.\n");
		return false;
	}

	// pmic callback
	static struct gpio_callback event_cb;

	gpio_init_callback(&event_cb, pmic_event_callback,
					   BIT(NPM13XX_EVENT_VBUS_DETECTED) |
						   BIT(NPM13XX_EVENT_VBUS_REMOVED) |
						   BIT(NPM13XX_EVENT_GPIO1_EDGE) |
						   BIT(NPM13XX_EVENT_GPIO0_EDGE) |
						   BIT(NPM13XX_EVENT_CHG_COMPLETED) |
						   BIT(NPM13XX_EVENT_CHG_ERROR));

	mfd_npm13xx_add_callback(pmic, &event_cb);

	// read initial vbus status
	struct sensor_value val;
	int ret = sensor_attr_get(charger, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS,
							  (enum sensor_attribute)SENSOR_ATTR_NPM13XX_CHARGER_VBUS_PRESENT,
							  &val);
	if (ret < 0)
	{
		return false;
	}

	vbus_connected = (val.val1 != 0) || (val.val2 != 0);
	printk("Initial Vbus status: %s\n", vbus_connected ? "connected" : "disconnected");

	/* Enable charging if driver is ready */

	return true;
}

static void pmic_event_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
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

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err)
	{
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err)
	{
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		return;
	}

	printk("Connected to the phone\n");
	current_conn = bt_conn_ref(conn);

	struct bt_le_conn_param param = {
		.interval_min = 15, // 30ms (24 * 1.25ms)
		.interval_max = 30, // 50ms (40 * 1.25ms)
		.latency = 0,
		.timeout = 400, // 4000ms (400 * 10ms)
	};

	int ret = bt_conn_le_param_update(conn, &param);
	if (ret)
	{
		printk("Failed to update conn params (err %d)\n", ret);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	START_MEASUREMENTS = false;
	PMIC_MEASUREMENTS_DONE = false;

	k_sem_reset(&SEM_PMIC_MEASURE);
	k_sem_reset(&SEM_BT_NUS_SEND);
	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

static void recycled_cb(void)
{
	printk("Disconnect is complete, started advertising\n");
	advertising_start();
}

int send_to_bt(const uint8_t *data, uint16_t len)
{
	int err = bt_nus_send(current_conn, data, len);
	if (err < 0)
	{
		printk("Failed to send data (err %d)\n", err);
		return err;
	}

	return 0;
}

static void auth_cancel(struct bt_conn *conn)
{
	printk("Pairing cancelled\n");
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
							 enum bt_security_err err)
{
	if (!err)
	{
		printk("Security changed: level %d\n", level);
	}
	else
	{
		printk("Security failed: level %d err %d\n", level, err);
	}
}

static void nus_receive_cb(struct bt_conn *conn,
						   const uint8_t *const data,
						   uint16_t len)
{
	printk("Received NUS data: len=%d\n", len);
	printk("Data: ");
	const uint8_t *ptr = data;
	const uint8_t *end = data + len;

	for (; ptr < end; ptr++)
	{
		printk("%c", *ptr);
	}
	printk("\n");
	// START/STOP commands
	if (len == 4 || len == 5)
	{
		if (strncmp((const char *)data, "START", 5) == 0)
		{
			printk("START command received\n");
			// start measurements
			START_MEASUREMENTS = true;
		}
		else if (strncmp((const char *)data, "STOP", 4) == 0)
		{
			printk("STOP command received\n");
			// stop measurements
			START_MEASUREMENTS = false;
		}
	}
}

static void send_enabled_cb(enum bt_nus_send_status status)
{
	if (status == BT_NUS_SEND_STATUS_ENABLED)
	{
		nus_tx_connected = true;
		printk("NUS notifications enabled\n");
		k_sem_give(&SEM_BT_NUS_SEND);
	}
	else
	{
		nus_tx_connected = false;
		printk("NUS notifications disabled\n");
		k_sem_reset(&SEM_PMIC_MEASURE); // Reset to not ready
		k_sem_reset(&SEM_BT_NUS_SEND);
	}
}

static ssize_t bt_read_battery_values(struct bt_conn *conn,
									  const struct bt_gatt_attr *attr,
									  void *buf, uint16_t len,
									  uint16_t offset)
{
	static struct sensor_value volt;
	static struct sensor_value current;
	static struct sensor_value error;
	static struct sensor_value status;
	static struct sensor_value vbus_present;

	sensor_sample_fetch(charger);
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &volt);
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &current);
	sensor_channel_get(charger, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_STATUS,
					   &status);
	sensor_channel_get(charger, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_ERROR, &error);
	sensor_attr_get(charger, (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS,
					(enum sensor_attribute)SENSOR_ATTR_NPM13XX_CHARGER_VBUS_PRESENT,
					&vbus_present);

	char buffer[6];
	int len_buf;

	if (attr->user_data == &pmic_measurement_data.voltage)
	{
		len_buf = snprintf(buffer, sizeof(buffer), "%d.%03d", volt.val1, volt.val2 / 1000);
		return bt_gatt_attr_read(conn, attr, buf, len, offset,
								 buffer, len_buf);
	}
	else if (attr->user_data == &pmic_measurement_data.current)
	{
		len_buf = snprintf(buffer, sizeof(buffer), "%03d", current.val2 / 1000);//mA
		return bt_gatt_attr_read(conn, attr, buf, len, offset,
								 buffer, len_buf);
	}
	else if (attr->user_data == &pmic_measurement_data.charger_status)
	{
		len_buf = snprintf(buffer, sizeof(buffer), "%d", status.val1);
		return bt_gatt_attr_read(conn, attr, buf, len, offset,
								 buffer, len_buf);
	}
	else if (attr->user_data == &pmic_measurement_data.charger_error)
	{
		len_buf = snprintf(buffer, sizeof(buffer), "%d", error.val1);
		return bt_gatt_attr_read(conn, attr, buf, len, offset,
								 buffer, len_buf);
	}
	else if (attr->user_data == &pmic_measurement_data.vbus_present)
	{
		len_buf = snprintf(buffer, sizeof(buffer), "%s", vbus_present.val1 ? "Y" : "N");
		return bt_gatt_attr_read(conn, attr, buf, len, offset,
								 buffer, len_buf);
	}
	else
	{
		buffer[0] = 'X';
		return bt_gatt_attr_read(conn, attr, buf, len, offset,
								 buffer, 1);
	}
}

static ssize_t bt_read_hr_rp_values(struct bt_conn *conn,
									const struct bt_gatt_attr *attr,
									void *buf, uint16_t len,
									uint16_t offset)
{
	printk("Heart rate and respiratory read requested\n");
	return 0;
}

#ifndef SENSORS_OFF
static bool configure_interrupts(void)
{
	// config sensor interrupts

	if (!gpio_is_ready_dt(&st1vafe3bx_int_specs))
	{
		printk("INT GPIO not ready\n");
		return -1;
	}
	if (!gpio_is_ready_dt(&st1vafe6ax_int_specs1))
	{
		printk("INT1 GPIO not ready\n");
		return -1;
	}
	if (!gpio_is_ready_dt(&st1vafe6ax_int_specs2))
	{
		printk("INT2 GPIO not ready\n");
		return -1;
	}

	int ret;
	// configure pins as input
	ret = gpio_pin_configure_dt(&st1vafe3bx_int_specs, GPIO_INPUT);
	if (ret < 0)
	{
		printk("Could not configure INT to INPUT GPIO\n");
		return false;
	}
	ret = gpio_pin_configure_dt(&st1vafe6ax_int_specs1, GPIO_INPUT);
	if (ret < 0)
	{
		printk("Could not configure INT1 to INPUT GPIO\n");
		return false;
	}
	ret = gpio_pin_configure_dt(&st1vafe6ax_int_specs2, GPIO_INPUT);
	if (ret < 0)
	{
		printk("Could not configure INT2 to INPUT GPIO\n");
		return false;
	}

	// configure interrupts

	// ret = gpio_pin_interrupt_configure_dt(&st1vafe3bx_int_specs, GPIO_INT_EDGE_TO_ACTIVE);
	// if (ret < 0)
	// {
	// 	printk("Could not configure INT GPIO(%d)\n", ret);
	// 	return false;
	// }
	ret = gpio_pin_interrupt_configure_dt(&st1vafe6ax_int_specs1, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0)
	{
		printk("Could not configure INT1 GPIO(%d)\n", ret);
		return false;
	}
	ret = gpio_pin_interrupt_configure_dt(&st1vafe6ax_int_specs2, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0)
	{
		printk("Could not configure INT2 GPIO(%d)\n", ret);
		return false;
	}

	// gpio_init_callback(&st1vafe3bx_interrupt, st1vafe3bx_interrupt_cb, BIT(st1vafe3bx_int_specs.pin));
	gpio_init_callback(&st1vafe6ax_interrupt1, st1vafe6ax_interrupt_cb1, BIT(st1vafe6ax_int_specs1.pin));
	gpio_init_callback(&st1vafe6ax_interrupt2, st1vafe6ax_interrupt_cb2, BIT(st1vafe6ax_int_specs2.pin));

	// gpio_add_callback(st1vafe3bx_int_specs.port, &st1vafe3bx_interrupt);
	gpio_add_callback(st1vafe6ax_int_specs1.port, &st1vafe6ax_interrupt1);
	gpio_add_callback(st1vafe6ax_int_specs2.port, &st1vafe6ax_interrupt2);

	return true;
}

static void st1vafe3bx_interrupt_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_mutex_lock(&MUTEX_RECEIVED_INTERRUPT, K_NO_WAIT);
	RECEIVED_INTERRUPT = true;
	k_mutex_unlock(&MUTEX_RECEIVED_INTERRUPT);
}
static void st1vafe6ax_interrupt_cb1(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_mutex_lock(&MUTEX_RECEIVED_INTERRUPT, K_NO_WAIT);
	RECEIVED_INTERRUPT = true;
	k_mutex_unlock(&MUTEX_RECEIVED_INTERRUPT);
}
static void st1vafe6ax_interrupt_cb2(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_mutex_lock(&MUTEX_RECEIVED_INTERRUPT, K_NO_WAIT);
	RECEIVED_INTERRUPT = true;
	k_mutex_unlock(&MUTEX_RECEIVED_INTERRUPT);
}

#endif