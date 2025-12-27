

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
#include <shell/shell_bt_nus.h>
#include <zephyr/bluetooth/hci.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/drivers/sensor.h>

#include <zephyr/drivers/regulator.h>
#include <zephyr/dt-bindings/regulator/npm13xx.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/drivers/mfd/npm13xx.h>

// include stm generic drivers
#include "st1vafe_wrapper.h"

// DEFINES
#define SENSORS_OFF
#define PMIC_OFF

// BLUETOOTH
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

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
static void pmic_event_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void read_sensors(void);
bool configure_events(void);
void pmic_measurements(void *arg1, void *arg2, void *arg3);
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
typedef enum
{
	BT_PKT_PMIC,
	BT_PKT_SENSOR,
	BT_PKT_STATUS,
} bt_pkt_type_t;

typedef struct
{
	bt_pkt_type_t type;
	uint8_t len;
	uint8_t payload[20];
} bt_packet_t;

static struct bt_nus_cb nus_cb = {
	.received = nus_receive_cb,
	.sent = NULL,
	.send_enabled = send_enabled_cb,
};

static const struct device *pmic = PMIC_DEVICE;
static const struct device *leds = PMIC_LEDS;
static const struct device *regulators = PMIC_REGULATORS;
static const struct device *charger = PMIC_CHARGER;
static const struct device *gpio = PMIC_GPIO;

static volatile bool vbus_connected;

// thread for power measurements
#define PMIC_STACK_SIZE 1024
#define PMIC_PRIORITY 8
K_THREAD_STACK_DEFINE(pmic_stack, PMIC_STACK_SIZE);
struct k_thread pmic_thread;

static struct bt_conn *current_conn;
static struct k_work adv_work;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

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

static K_SEM_DEFINE(bt_nus_ready, 0, 1);

int main(void)
{
	int err;
#ifndef SENSORS_OFF

	uint8_t dummy = 0xAA;
	st1vafe_init();
	st1vafe3bx_device_id_get(&st1vafe3bx_ctx, &dummy);

#endif

#ifndef PMIC_OFF
	printk("PMIC/NPM1300 off\n");
	if (!configure_events())
	{
		printk("Error: could not configure events\n");
		return 0;
	}
	printk("PMIC device ok\n");
#endif

	// start pmic measurement thread after 1 second
	k_thread_create(&pmic_thread, pmic_stack,
					PMIC_STACK_SIZE,
					pmic_measurements,
					NULL, NULL, NULL,
					PMIC_PRIORITY, 0,
					K_MSEC(1000));

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

	// communication between phone
	err = bt_nus_init(&nus_cb);
	if (err)
	{
		printk("Failed to initialize BT NUS shell (err: %d)\n", err);
		return 0;
	}

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

	return 0; // main thread completed
}
void pmic_measurements(void *arg1, void *arg2, void *arg3)
{

	printk("Waiting for NUS to be enabled...\n");

	// Block until first client enables notifications
	k_sem_take(&bt_nus_ready, K_FOREVER);
	k_sem_give(&bt_nus_ready);
	while (1)
	{
		// read_sensors();
		static uint8_t count = 0;
		count += 11;

		if (current_conn)
		{
			if (k_sem_take(&bt_nus_ready, K_NO_WAIT) == 0)
			{
				int err = send_to_bt((const uint8_t *)&count, sizeof(count));
				k_sem_give(&bt_nus_ready);
				if (err)
				{
					printk("Send failed: %d\n", err);
				}
			}
		}
		k_msleep(UPDATE_TIME_MS);

		// also send through bluetooth
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
	int ret = sensor_attr_get(charger, SENSOR_CHAN_CURRENT, SENSOR_ATTR_UPPER_THRESH, &val);
	if (ret < 0)
	{
		return false;
	}

	vbus_connected = (val.val1 != 0) || (val.val2 != 0);
	printk("Initial Vbus status: %s\n", vbus_connected ? "connected" : "disconnected");
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

	printk("Connected\n");
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

	k_sem_reset(&bt_nus_ready);
	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

static void recycled_cb(void)
{
	printk("Disconnect is complete!\n");
	advertising_start();
}

int send_to_bt(const uint8_t *data, uint16_t len)
{
	if (!current_conn)
	{
		printk("Not connected\n");
		return -ENOTCONN;
	}

	int err = bt_nus_send(current_conn, data, len);
	if (err)
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
	switch (err)
	{
	case BT_SECURITY_ERR_SUCCESS:
		printk("  Success\n");
		break;
	case BT_SECURITY_ERR_AUTH_FAIL:
		printk("  Authentication failed\n");
		break;
	case BT_SECURITY_ERR_PIN_OR_KEY_MISSING:
		printk("  PIN/Key missing\n");
		break;
	case BT_SECURITY_ERR_OOB_NOT_AVAILABLE:
		printk("  OOB not available\n");
		break;
	case BT_SECURITY_ERR_AUTH_REQUIREMENT:
		printk("  Authentication requirement not met\n");
		break;
	case BT_SECURITY_ERR_PAIR_NOT_SUPPORTED:
		printk("  Pairing not supported\n");
		break;
	case BT_SECURITY_ERR_PAIR_NOT_ALLOWED:
		printk("  Pairing not allowed\n");
		break;
	case BT_SECURITY_ERR_INVALID_PARAM:
		printk("  Invalid parameter\n");
		break;
	default:
		printk("  Unknown error: %d\n", err);
		break;
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
}

static void send_enabled_cb(enum bt_nus_send_status status)
{
	if (status == BT_NUS_SEND_STATUS_ENABLED)
	{
		printk("NUS notifications enabled\n");
		k_sem_give(&bt_nus_ready); // Signal ready
	}
	else
	{
		printk("NUS notifications disabled\n");
		k_sem_reset(&bt_nus_ready); // Reset to not ready
	}
}