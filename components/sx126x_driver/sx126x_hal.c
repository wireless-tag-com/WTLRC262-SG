#include <stdio.h>
#include <string.h>
#include "stdint.h"

#include "sx126x_hal.h"
#include "sx126x_regs.h"
#include "sx126x.h"

#include "driver/gpio.h"
#include "freertos/task.h"
#include "esp_log.h"

// #define SPI_DEBUG

#define RADIO_DIOX_PIN 3 // Interrupt input pins
#define RADIO_BUSY_PIN 4 // Busy state input pin, low indicates that the command can be received
#define RADIO_NRST_PIN 5 // reset pin, low active
#define RADIO_SPI_MISO_PIN 6
#define RADIO_SPI_MOSI_PIN 7
#define RADIO_SPI_SCLK_PIN 10
#define RADIO_NSS_PIN 18

#define TAG "SX126X_HAL"

struct spi_context spi_context_t;
volatile bool irq_fired_flag = false; // Indicates the interrupt trigger flag of sx126x
static int isr_trigger_count = 0;    // Number of times an sx126x interrupt was triggered

typedef struct
{
    spi_device_handle_t *spi;
    gpio_num_t nss;
    gpio_num_t reset;
    gpio_num_t irq;
    gpio_num_t busy;
} sx126x_hal_context_t;

static sx126x_hal_context_t sx126x_pins = {
    .spi = &spi_context_t.spi,
    .nss = RADIO_NSS_PIN,
    .reset = RADIO_NRST_PIN,
    .irq = RADIO_DIOX_PIN,
    .busy = RADIO_BUSY_PIN};

spi_bus_config_t buscfg = {
    .miso_io_num = RADIO_SPI_MISO_PIN,
    .mosi_io_num = RADIO_SPI_MOSI_PIN,
    .sclk_io_num = RADIO_SPI_SCLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1};

spi_device_interface_config_t devcfg = {
    .clock_speed_hz = SPI_SPEED,
    .mode = 0,
    .spics_io_num = -1, // no CS pin
    .queue_size = 2,
    .flags = 0,
    .pre_cb = NULL};


/************************************function************************************/

/**
 * @brief Interrupt handler for the irq pin
 */
static void gpio_isr_handler(void *arg)
{
    irq_fired_flag = true;
    isr_trigger_count++;
}

static void gpio_task_example(void *arg)
{
    while (1)
    {
        printf("isr_trigger_count = %d\n", isr_trigger_count);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void sx126x_gpio_init(void)
{
    gpio_config_t gpio_conf;

    // DIOX Pin
    gpio_conf.intr_type = GPIO_INTR_POSEDGE; // GPIO_INTR_DISABLE  GPIO_INTR_POSEDGE
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pin_bit_mask = (1 << RADIO_DIOX_PIN);
    gpio_conf.pull_down_en = 0;
    gpio_conf.pull_up_en = 1;
    gpio_config(&gpio_conf);

    // BUSY Pin
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pin_bit_mask = (1 << RADIO_BUSY_PIN);
    gpio_conf.pull_down_en = 0;
    gpio_conf.pull_up_en = 1;
    gpio_config(&gpio_conf);

    // NSS Pin
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (1 << RADIO_NSS_PIN);
    gpio_conf.pull_down_en = 0;
    gpio_conf.pull_up_en = 0;
    gpio_config(&gpio_conf);

    // NRST Pin
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (1 << RADIO_NRST_PIN);
    gpio_conf.pull_down_en = 0;
    gpio_conf.pull_up_en = 0;
    gpio_config(&gpio_conf);

    gpio_set_level(sx126x_pins.nss, 1);
    gpio_set_level(sx126x_pins.reset, 1);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(RADIO_DIOX_PIN, gpio_isr_handler, (void *)RADIO_DIOX_PIN);
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    // ESP_LOGI(TAG, "gpio initialization is completed\n");
}

/**
 * @brief Initialize the spi
 */
static void sx126x_spi_init(void)
{
    esp_err_t ret;

    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_context_t.spi);
    ESP_ERROR_CHECK(ret);

    // ESP_LOGI(TAG, "spi initialization is completed\n");
}

/**
 * @brief Wait until radio busy pin is reset to 0
 */
static void sx126x_hal_wait_on_busy(void)
{
    // printf("wait_on_busy()...");
    while (gpio_get_level(sx126x_pins.busy) == 1)
    {
        ;
    }
}

/**
 * @brief spi data transfer
 *
 * @remark Must be implemented by the upper layer
 * @remark Do not forget to call \ref sx126x_hal_wakeup function at the very
 * beginning of the implementation.
 *
 * @param [in] spi          The spi handle used to complete data transfer
 * @param [in] len          The length of the transmitted data
 * @param [in] src          Data sent by the mcu to sx126x, If the value is NULL, no send is required
 * @param [in] dest         Data read by the mcu from the sx126x, If the value is NULL, no reading is required
 *
 * @retval  none
 */
static void esp32_spi_transfer(spi_device_handle_t *spi, size_t len, const uint8_t *src, uint8_t *dest)
{
    int bits_to_send = len * 8;

    if (!bits_to_send)
    {
        ESP_LOGW(TAG, "buffer too short\n");
        return;
    }

    if (len <= 4)
    {
        spi_transaction_t transaction = {0};

        if (src != NULL)
        {
            memcpy(&transaction.tx_data, src, len);
        }

        transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        transaction.length = bits_to_send;
        spi_device_transmit(*spi, &transaction);

        if (dest != NULL)
        {
            memcpy(dest, &transaction.rx_data, len);
        }
    }
    else
    {
        int offset = 0;
        int bits_remaining = bits_to_send;
        int max_transaction_bits = SPI_MAX_XFER_BITS;
        spi_transaction_t *transaction, *result, transactions[2];
        int i = 0;

        spi_device_acquire_bus(*spi, portMAX_DELAY);

        while (bits_remaining)
        {
            transaction = transactions + i++ % 2;
            memset(transaction, 0, sizeof(spi_transaction_t));

            transaction->length =
                bits_remaining > max_transaction_bits ? max_transaction_bits : bits_remaining;

            if (src != NULL)
            {
                transaction->tx_buffer = src + offset;
            }
            if (dest != NULL)
            {
                transaction->rx_buffer = dest + offset;
            }

            spi_device_queue_trans(*spi, transaction, portMAX_DELAY);
            bits_remaining -= transaction->length;

            if (offset > 0)
            {
                // wait for previously queued transaction
                spi_device_get_trans_result(*spi, &result, portMAX_DELAY);
            }

            // doesn't need ceil(); loop ends when bits_remaining is 0
            offset += transaction->length / 8;
        }

        // wait for last transaction
        spi_device_get_trans_result(*spi, &result, portMAX_DELAY);
        spi_device_release_bus(*spi);
    }
}

void esp32_spi_wb(spi_device_handle_t *spi, const uint8_t *data, uint16_t size)
{
#ifdef SPI_DEBUG
    printf("spi write data (len = %d): ", size);
    for (int i = 0; i < size; i++)
        printf("0x%02x ", data[i]);
    printf("\n");
#endif
    esp32_spi_transfer(spi, size, data, NULL);
}

void esp32_spi_rb(spi_device_handle_t *spi, uint8_t *data, uint16_t size)
{
#ifdef SPI_DEBUG
    printf("spi read data (len = %d): ", size);
    for (int i = 0; i < size; i++)
        printf("0x%02x ", data[i]);
    printf("\n");
#endif

    esp32_spi_transfer(spi, size, NULL, data);
}

void sx126x_gpio_spi_init(void)
{
    sx126x_gpio_init();
    sx126x_spi_init();
}

uint8_t sx126x_get_irq_pin_status(const void *context)
{
    return gpio_get_level(sx126x_pins.irq);
}

sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,
                                     const uint8_t *data, const uint16_t data_length)
{
    sx126x_hal_wait_on_busy();

    gpio_set_level(sx126x_pins.nss, 0);
    if (command_length > 0)
        esp32_spi_wb(&spi_context_t.spi, command, command_length);
    if (data_length > 0)
        esp32_spi_wb(&spi_context_t.spi, data, data_length);
    gpio_set_level(sx126x_pins.nss, 1);

    // 0x84 - SX128X_SET_SLEEP opcode. In sleep mode the radio dio is struck to 1 => do not test it
    if (command[0] != 0x84)
    {
        sx126x_hal_wait_on_busy();
    }

    return SX128X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,
                                    uint8_t *data, const uint16_t data_length)
{
    sx126x_hal_wait_on_busy();

    gpio_set_level(sx126x_pins.nss, 0);
    esp32_spi_wb(&spi_context_t.spi, command, command_length);
    esp32_spi_rb(&spi_context_t.spi, data, data_length);
    gpio_set_level(sx126x_pins.nss, 1);

    return SX128X_HAL_STATUS_OK;
}

static void wait_ms(unsigned long ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

sx126x_hal_status_t sx126x_hal_reset(const void *context)
{
    gpio_set_level(sx126x_pins.reset, 0); // LOW
    wait_ms(5);
    gpio_set_level(sx126x_pins.reset, 1); // HIGH
    wait_ms(5);

    return SX128X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void *context)
{
    const uint8_t get_status_cmd[2] = {0xc0, 0x00};

    esp32_spi_wb(&spi_context_t.spi, get_status_cmd, 2);
    sx126x_hal_wait_on_busy();

    return SX128X_HAL_STATUS_OK;
}
