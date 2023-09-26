#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include "esp_log.h"

#include "sx126x.h"
#include "sx126x_hal.h"
#include "transplant.h"
#include "apps_configuration.h"


/* --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Size of ping-pong message prefix
 *
 * Expressed in bytes
 */
#define PING_PONG_PREFIX_SIZE 4

/**
 * @brief Threshold:number of exchanges before informing user on UART that the board pair is still not synchronized
 *
 * Expressed in number of packet exchanged
 */
#define SYNC_PACKET_THRESHOLD 64

/**
 * @brief Number of exchanges are stored in the payload of the packet exchanged during this PING-PONG communication
 *        this constant indicates where in the packet the two bytes used to count are located
 *
 * Expressed in bytes
 */
#define ITERATION_INDEX ( PING_PONG_PREFIX_SIZE + 1 )

/**
 * @brief Duration of the wait between each ping-pong activity, can be used to adjust ping-pong speed
 *
 * Expressed in milliseconds
 */
#define DELAY_PING_PONG_PACE_MS 200

/**
 * @brief Duration of the wait before packet transmission to assure reception status is ready on the other side
 *
 * Expressed in milliseconds
 */
#define DELAY_BEFORE_TX_MS 20

#ifndef RX_TIMEOUT_VALUE
#define RX_TIMEOUT_VALUE 600
#endif

#define TAG "main"

static uint8_t buffer_tx[PAYLOAD_LENGTH];
static bool    is_master = true;

const uint8_t ping_msg[PING_PONG_PREFIX_SIZE] = "PING";
const uint8_t pong_msg[PING_PONG_PREFIX_SIZE] = "PONG";

static uint16_t iteration       = 0;
static uint16_t packets_to_sync = 0;


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

extern struct spi_context spi_context_t;

extern void apps_common_sx126x_init( const void* context );
extern void apps_common_sx126x_radio_init( const void* context );
extern void apps_common_sx126x_irq_process( const void* context );
extern uint32_t get_time_on_air_in_ms( void );

void on_tx_done( void );
void on_rx_done( void );
void apps_common_sx126x_receive( const void* context, uint8_t* buffer, uint8_t* size, uint8_t max_size );

/**
 * @brief Print the sent data, delay, and set the rx mode
 */
void on_tx_done( void )
{
    printf( "Sent message {%s}, iteration %d\n", buffer_tx, iteration );
    vTaskDelay(DELAY_PING_PONG_PACE_MS / portTICK_PERIOD_MS);
    sx126x_set_rx( (void*)(&spi_context_t), 0 );
}

/**
 * @brief Read and compare the data in rxbuf, send the data again according to the comparison result, set the tx mode
 */
void on_rx_done( void )
{
    uint8_t buffer_rx[PAYLOAD_LENGTH];
    uint8_t size;

    packets_to_sync = 0;

    apps_common_sx126x_receive( (void*)(&spi_context_t), buffer_rx, &size, PAYLOAD_LENGTH );
    iteration = buffer_rx[ITERATION_INDEX];

    iteration++;
    printf( "Received message {%s}, iteration %d\n", buffer_rx, iteration );

    if( is_master == true )
    {
        if( memcmp( buffer_rx, ping_msg, PING_PONG_PREFIX_SIZE ) == 0 )
        {
            is_master = false;
            memcpy( buffer_tx, pong_msg, PING_PONG_PREFIX_SIZE );
        }
        else if( memcmp( buffer_rx, pong_msg, PING_PONG_PREFIX_SIZE ) != 0 )
        {
            ESP_LOGE(TAG, "Unexpected message - PONG expected\n" );
        }
    }
    else
    {
        if( memcmp( buffer_rx, ping_msg, PING_PONG_PREFIX_SIZE ) != 0 )
        {
            ESP_LOGE(TAG, "Unexpected message\n" );

            is_master = true;
            memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
        }
    }

    vTaskDelay((DELAY_PING_PONG_PACE_MS + DELAY_BEFORE_TX_MS) / portTICK_PERIOD_MS);
    buffer_tx[ITERATION_INDEX] = ( uint8_t ) ( iteration );

    sx126x_write_buffer( (void*)(&spi_context_t), 0, buffer_tx, PAYLOAD_LENGTH );

    sx126x_set_tx( (void*)(&spi_context_t), 0 );
}

/**
 * @brief Read data from the current rxbuf
 */
void apps_common_sx126x_receive( const void* context, uint8_t* buffer, uint8_t* size, uint8_t max_size )
{
    sx126x_rx_buffer_status_t rx_buffer_status;
    sx126x_pkt_status_lora_t  pkt_status_lora;
    sx126x_pkt_status_gfsk_t  pkt_status_gfsk;

    sx126x_get_rx_buffer_status( context, &rx_buffer_status );

    if( max_size < rx_buffer_status.pld_len_in_bytes )
    {
        ESP_LOGE( TAG, "Received more bytes than expected (%d vs %d), reception in buffer cancelled.\n",
                             rx_buffer_status.pld_len_in_bytes, max_size );
        *size = 0;
    }
    else
    {
        sx126x_read_buffer( context, 0, buffer, rx_buffer_status.pld_len_in_bytes );
        *size = rx_buffer_status.pld_len_in_bytes;
    }

    HAL_DBG_TRACE_ARRAY( "Packet content", buffer, *size );

    printf( "Packet status:\n" );
    if( PACKET_TYPE == SX126X_PKT_TYPE_LORA )
    {
        sx126x_get_lora_pkt_status( context, &pkt_status_lora );
        printf( "  - RSSI packet = %i dBm\n", pkt_status_lora.rssi_pkt_in_dbm );
        printf( "  - Signal RSSI packet = %i dBm\n", pkt_status_lora.signal_rssi_pkt_in_dbm );
        printf( "  - SNR packet = %i dB\n", pkt_status_lora.snr_pkt_in_db );
    }
    else if( PACKET_TYPE == SX126X_PKT_TYPE_GFSK )
    {
        sx126x_get_gfsk_pkt_status( context, &pkt_status_gfsk );
        printf( "  - RSSI average = %i dBm\n", pkt_status_gfsk.rssi_avg );
        printf( "  - RSSI sync = %i dBm\n", pkt_status_gfsk.rssi_sync );
    }
}


void app_main(void)
{
    sx126x_gpio_spi_init();

    apps_common_sx126x_init( (void*)(&spi_context_t) );
    
    apps_common_sx126x_radio_init( (void*)(&spi_context_t) );

    sx126x_set_dio_irq_params(
        (void*)(&spi_context_t), SX126X_IRQ_ALL,
        SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_CRC_ERROR,
        SX126X_IRQ_NONE, SX126X_IRQ_NONE );

    sx126x_clear_irq_status( (void*)(&spi_context_t), SX126X_IRQ_ALL );

    // fill buffer_tx
    memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
    buffer_tx[PING_PONG_PREFIX_SIZE] = 0;
    buffer_tx[ITERATION_INDEX]       = ( uint8_t ) ( iteration );
    for( int i = PING_PONG_PREFIX_SIZE + 1 + 1; i < PAYLOAD_LENGTH; i++ )
    {
        buffer_tx[i] = i;
    }

    sx126x_write_buffer( (void*)(&spi_context_t), 0, buffer_tx, PAYLOAD_LENGTH );
    
    sx126x_set_tx( (void*)(&spi_context_t), 0 );

    while (1) {
        apps_common_sx126x_irq_process( (void*)(&spi_context_t) );
        // vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
