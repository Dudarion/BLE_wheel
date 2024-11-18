#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/services/hids.h>
#include <logging/log.h>

#define LED_PIN  4  
#define L_BTN_PIN  5  
#define R_BTN_PIN  6 

// I2C configuration
#define BNO055_ADDRESS 0x28
#define BNO055_ACC_DATA_X_LSB_ADDR 0x08
#define I2C_INSTANCE_ID 0

// I2C instance
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(I2C_INSTANCE_ID);

// UART instance
static const nrf_drv_uart_t m_uart = NRF_DRV_UART_INSTANCE(0);

// Function for initializing the GPIO pin
void gpio_init(void) {
    NRF_P0->DIRSET = (1 << LED_PIN);  // Set the direction of the pin to output
    NRF_P0->OUTCLR = (1 << LED_PIN);  // Turn off the LED initially
}

// Function for initializing the I2C interface
void i2c_init(void) {
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 27,   // Change to your SCL pin
       .sda                = 26,   // Change to your SDA pin
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

// Function for initializing the UART interface
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


LOG_MODULE_REGISTER(main);

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

static int read_imu(uint8_t reg, uint8_t *data, uint16_t length)
{
    uint8_t addr = DT_REG_ADDR(DT_NODELABEL(bno055));
    return i2c_write_read(i2c_dev, addr, &reg, 1, data, length);
}

// Function for reading accelerometer data from BNO055
void read_accelerometer_data(int16_t *accel_data) {
    ret_code_t err_code;
    uint8_t reg = BNO055_ACC_DATA_X_LSB_ADDR;
    uint8_t buffer[6];

    err_code = nrf_drv_twi_tx(&m_twi, BNO055_ADDRESS, &reg, 1, true);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_rx(&m_twi, BNO055_ADDRESS, buffer, 6);
    APP_ERROR_CHECK(err_code);

    accel_data[0] = (buffer[1] << 8) | buffer[0]; // X-axis
    accel_data[1] = (buffer[3] << 8) | buffer[2]; // Y-axis
    accel_data[2] = (buffer[5] << 8) | buffer[4]; // Z-axis
}



void main(void)
{
    int err;

    led_dev = device_get_binding(DT_LABEL(LED0_NODE));
    if (led_dev == NULL) {
        LOG_ERR("Didn't find LED device %s", DT_LABEL(LED0_NODE));
        return;
    }

    gpio_pin_configure(led_dev, DT_GPIO_PIN(LED0_NODE, gpios), GPIO_OUTPUT_ACTIVE);

    i2c_dev = device_get_binding(DT_LABEL(I2C_DEV_NODE));
    if (i2c_dev == NULL) {
        LOG_ERR("Didn't find I2C device %s", DT_LABEL(I2C_DEV_NODE));
        return;
    }

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");

    hids_init();

    while (1) {
        read_accelerometer_data(accel_data);
        
        snprintf(uart_buffer, sizeof(uart_buffer), "Accel X: %d, Y: %d, Z: %d\r\n", accel_data[0], accel_data[1], accel_data[2]);
        uart_send_string(uart_buffer);

        clamp_imu();
        gp.x       = -(imu_arr[1]/4); 
        gp.y       = -(imu_arr[2]/4); 
        gp.z       = -(imu_arr[0]/4);
        gp.rz      = 127;
        gp.rx      = 0;
        gp.ry      = 0;
        gp.hat     = 0;
        gp.buttons = (bt_left << 8) | (bt_right << 9);
        blegamepad.report(&gp);

        blink_led();
    }
}
