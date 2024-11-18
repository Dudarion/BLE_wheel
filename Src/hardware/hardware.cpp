#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_timer.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "ble.h"
#include "ble_hids.h"
#include "nrf_delay.h"
#include "app_error.h"

// Define GPIO Pins
#define BUTTON1_PIN 13   // GPIO pin for Button 1
#define BUTTON2_PIN 14   // GPIO pin for Button 2
#define LED_PIN     17   // GPIO pin for LED

// TWI (I2C) configuration for BNO055
#define TWI_INSTANCE_ID     0
#define BNO055_ADDR         0x28 // 7-bit I2C address of BNO055
#define BNO055_ACCEL_DATA   0x08 // Register to read accelerometer data

// Timer Interval
#define NOTIFICATION_INTERVAL_MS 20

// BLE Configuration
#define DEVICE_NAME "science rock"

// Global Variables
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static uint8_t m_notification_data[5];

// Function Prototypes
void twi_init(void);
void gpio_init(void);
void ble_init(void);
void bno055_init(void);
void read_bno055_data(uint8_t *data_buffer);
void send_ble_notification(void);

void gpio_init(void) {
    ret_code_t err_code;

    // Initializing GPIO
    if (!nrf_drv_gpiote_is_init()) {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    // Configuring buttons as inputs
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(BUTTON1_PIN, &config, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(BUTTON1_PIN, true);

    err_code = nrf_drv_gpiote_in_init(BUTTON2_PIN, &config, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(BUTTON2_PIN, true);

    // Configure LED as output
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_pin_clear(LED_PIN);  // Turn off LED initially
}

void uart_init(void) {
    ret_code_t err_code;

    const nrf_drv_uart_config_t uart_config = {
        .pseltxd            = 6,   // Change to your TXD pin
        .pselrxd            = 8,   // Change to your RXD pin
        .pselcts            = NRF_UART_PSEL_DISCONNECTED,
        .pselrts            = NRF_UART_PSEL_DISCONNECTED,
        .p_context          = NULL,
        .hwfc               = NRF_UART_HWFC_DISABLED,
        .parity             = NRF_UART_PARITY_EXCLUDED,
        .baudrate           = NRF_UART_BAUDRATE_115200
    };

    err_code = nrf_drv_uart_init(&m_uart, &uart_config, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_uart_tx_enable(&m_uart);
}

// Function for sending data over UART
void uart_send_string(const char *str) {
    nrf_drv_uart_tx(&m_uart, (uint8_t *)str, strlen(str));
}

void twi_init(void) {
    ret_code_t err_code;

    // TWI (I2C) configuration
    const nrf_drv_twi_config_t twi_config = {
        .scl                = 26,  // SCL Pin
        .sda                = 27,  // SDA Pin
        .frequency          = NRF_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void bno055_init(void) {
    uint8_t id_register = 0x00; // ID register
    uint8_t chip_id = 0;

    // Read chip ID to verify communication
    nrf_drv_twi_tx(&m_twi, BNO055_ADDR, &id_register, 1, true);
    nrf_drv_twi_rx(&m_twi, BNO055_ADDR, &chip_id, 1);

    if (chip_id != 0xA0) {
        NRF_LOG_INFO("BNO055 Initialization Failed!");
    } else {
        NRF_LOG_INFO("BNO055 Initialized Successfully");
    }
}

void read_bno055_data(uint8_t *data_buffer) {
    uint8_t reg = BNO055_ACCEL_DATA; // Start register for accelerometer data (6 bytes)

    // Request accelerometer and gyro data
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, BNO055_ADDR, &reg, 1, true);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_rx(&m_twi, BNO055_ADDR, data_buffer, 6);
    APP_ERROR_CHECK(err_code);
}

static struct bt_hids hids_obj;

/* HID Report Map for a gamepad */
static const uint8_t hid_report_map[] = {
    0x05, 0x01,     // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,     // USAGE (Gamepad)
    0xa1, 0x01,     // COLLECTION (Application)
    0x05, 0x09,     //   USAGE_PAGE (Button)
    0x19, 0x01,     //   USAGE_MINIMUM (Button 1)
    0x29, 0x10,     //   USAGE_MAXIMUM (Button 16)
    0x15, 0x00,     //   LOGICAL_MINIMUM (0)
    0x25, 0x01,     //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,     //   REPORT_SIZE (1)
    0x95, 0x10,     //   REPORT_COUNT (16)
    0x81, 0x02,     //   INPUT (Data,Var,Abs)
    0x05, 0x01,     //   USAGE_PAGE (Generic Desktop)
    0x09, 0x30,     //   USAGE (X)
    0x09, 0x31,     //   USAGE (Y)
    0x09, 0x32,     //   USAGE (Z)
    0x09, 0x35,     //   USAGE (Rz)
    0x15, 0x81,     //   LOGICAL_MINIMUM (-127)
    0x25, 0x7f,     //   LOGICAL_MAXIMUM (127)
    0x75, 0x08,     //   REPORT_SIZE (8)
    0x95, 0x04,     //   REPORT_COUNT (4)
    0x81, 0x02,     //   INPUT (Data,Var,Abs)
    0xc0            // END_COLLECTION
};


static void hid_init(void)
{
    char device_name[] = "Science_rock";
    struct bt_hids_init_param hids_init_obj = {
        .info = {
            .bcd_hid = 0x0111,
            .b_country_code = 0x00,
            .flags = BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE,
        },
        .rep_map = {
            .data = hid_report_map,
            .size = sizeof(hid_report_map)
        },
        .report_input = {
            .max_len = 16
        }
    };

    bt_hids_init(&hids_obj, &hids_init_obj);
}

void send_ble_notification(void) {
    NRF_LOG_INFO("Sending BLE Notification: %02X %02X %02X %02X %02X", 
                 m_notification_data[0], m_notification_data[1], m_notification_data[2],
                 m_notification_data[3], m_notification_data[4]);
}

int main(void) {
    // Initialize Logging
    NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // Initialize GPIO, TWI (I2C), BLE, and BNO055
    gpio_init();
    uart_init();
    twi_init();
    bno055_init();
    ble_init();

    NRF_LOG_INFO("Initialization Complete, Entering Main Loop");

    while (true) {
        // Read accelerometer and gyro data from BNO055
        uint8_t sensor_data[6];
        read_bno055_data(sensor_data);
        int16_t euler[3];
        calculateEulerAngles(sensor_data, euler);

        // Read button states
        uint8_t button1_state = !nrf_gpio_pin_read(BUTTON1_PIN);
        uint8_t button2_state = !nrf_gpio_pin_read(BUTTON2_PIN);

        // Preparing notification data
        m_notification_data[0] = euler[0];  
        m_notification_data[1] = euler[1];  
        m_notification_data[2] = button1_state;   
        m_notification_data[3] = button2_state;   
        m_notification_data[4] = euler[2];  

        // Sending the 5-byte packet through BLE by notification
        send_ble_notification();

        // 20ms delay
        nrf_delay_ms(20);
    }
}
