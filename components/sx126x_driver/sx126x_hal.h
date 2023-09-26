#ifndef SX128X_HAL_H
#define SX128X_HAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>
#include "driver/spi_master.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */
#define SPI_SPEED 2000000 // 2M
#define SPI_MAX_XFER_BYTES 4092
#define SPI_MAX_XFER_BITS (SPI_MAX_XFER_BYTES * 8) // Has to be an even multiple of 8

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Write this to SPI bus while reading data, or as a dummy/placeholder
 */
#define SX126X_NOP (0x00)

    /*
     * -----------------------------------------------------------------------------
     * --- PUBLIC TYPES ------------------------------------------------------------
     */
    typedef enum sx126x_hal_status_e
    {
        SX128X_HAL_STATUS_OK = 0,
        SX128X_HAL_STATUS_ERROR = 3,
    } sx126x_hal_status_t;

    struct spi_context
    {
        spi_device_handle_t spi; // SPI device handle
    };


    /*
     * -----------------------------------------------------------------------------
     * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
     */

    /**
     * @brief Initialize the pins on esp32c2 that are connected to sx1262
     *
     * @param [in]  none
     *
     * @retval      none
     */
    void sx126x_gpio_spi_init(void);

    /**
     * @brief Radio data transfer - write
     *
     * @remark Must be implemented by the upper layer
     * @remark Do not forget to call \ref sx126x_hal_wakeup function at the very
     * beginning of the implementation.
     *
     * @param [in] context          Radio implementation parameters
     * @param [in] command          Pointer to the buffer to be transmitted
     * @param [in] command_length   Buffer size to be transmitted
     * @param [in] data             Pointer to the buffer to be transmitted
     * @param [in] data_length      Buffer size to be transmitted
     *
     * @retval sx126x_hal_status_t     Operation status
     */
    sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,
                                         const uint8_t *data, const uint16_t data_length);

    /**
     * @brief Radio data transfer - read
     *
     * @remark Must be implemented by the upper layer
     * @remark Do not forget to call \ref sx126x_hal_wakeup function at the very
     * beginning of the implementation.
     *
     * @param [in] context          Radio implementation parameters
     * @param [in] command          Pointer to the buffer to be transmitted
     * @param [in] command_length   Buffer size to be transmitted
     * @param [in] data             Pointer to the buffer to be received
     * @param [in] data_length      Buffer size to be received
     *
     * @retval sx126x_hal_status_t     Operation status
     */
    sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,
                                        uint8_t *data, const uint16_t data_length);

    /**
     * @brief Reset the radio
     *
     * @remark Must be implemented by the upper layer
     *
     * @param [in] context Radio implementation parameters
     *
     * @retval sx126x_hal_status_t    Operation status
     */
    sx126x_hal_status_t sx126x_hal_reset(const void *context);

    /**
     * @brief Wake the radio up.
     *
     * @remark Must be implemented by the upper layer
     *
     * @param [in] context Radio implementation parameters

     * @retval sx126x_hal_status_t    Operation status
     */
    sx126x_hal_status_t sx126x_hal_wakeup(const void *context);

    /**
     * @brief Gets the status of the irq pin
     *
     * @remark Must be implemented by the upper layer
     * @remark Do not forget to call \ref sx126x_hal_wakeup function at the very
     * beginning of the implementation.
     *
     * @param [in] context          Radio implementation parameters
     *
     * @retval The level of the irq pin, 0 is low, 1 is high
     */
    uint8_t sx126x_get_irq_pin_status(const void *context);

    /**
     * @brief spi write data
     *
     * @remark Must be implemented by the upper layer
     * @remark Do not forget to call \ref sx126x_hal_wakeup function at the very
     * beginning of the implementation.
     *
     * @param [in] spi       spi handle
     * @param [in] data      data to write
     * @param [in] size      Length of data to be written
     * @retval none
     */
    void esp32_spi_wb(spi_device_handle_t *spi, const uint8_t *data, uint16_t size);

    /**
     * @brief spi read data
     *
     * @remark Must be implemented by the upper layer
     * @remark Do not forget to call \ref sx126x_hal_wakeup function at the very
     * beginning of the implementation.
     *
     * @param [in] spi           spi handle
     * @param [in] data          data to read
     * @param [in] size          Length of data to be read
     * @retval none
     */
    void esp32_spi_rb(spi_device_handle_t *spi, uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif // SX128X_HAL_H

/* --- EOF ------------------------------------------------------------------ */
