#ifndef __TRANSPLANT_H__
#define __TRANSPLANT_H__

#include "stdint.h"
#include "sx126x.h"

#define SMTC_SHIELD_SX126X_FREQ_MIN 150000000
#define SMTC_SHIELD_SX126X_FREQ_MAX 960000000

#define SMTC_SHIELD_SX1261_MIN_PWR -17
#define SMTC_SHIELD_SX1261_MAX_PWR 15

#define SMTC_SHIELD_SX1262_MIN_PWR -9
#define SMTC_SHIELD_SX1262_MAX_PWR 22

#define SMTC_SHIELD_SX1268_MIN_PWR -9
#define SMTC_SHIELD_SX1268_MAX_PWR 22


typedef enum sx126x_commands_e
{
    // Operational Modes Functions
    SX126X_SET_SLEEP                  = 0x84,
    SX126X_SET_STANDBY                = 0x80,
    SX126X_SET_FS                     = 0xC1,
    SX126X_SET_TX                     = 0x83,
    SX126X_SET_RX                     = 0x82,
    SX126X_SET_STOP_TIMER_ON_PREAMBLE = 0x9F,
    SX126X_SET_RX_DUTY_CYCLE          = 0x94,
    SX126X_SET_CAD                    = 0xC5,
    SX126X_SET_TX_CONTINUOUS_WAVE     = 0xD1,
    SX126X_SET_TX_INFINITE_PREAMBLE   = 0xD2,
    SX126X_SET_REGULATOR_MODE         = 0x96,
    SX126X_CALIBRATE                  = 0x89,
    SX126X_CALIBRATE_IMAGE            = 0x98,
    SX126X_SET_PA_CFG                 = 0x95,
    SX126X_SET_RX_TX_FALLBACK_MODE    = 0x93,
    // Registers and buffer Access
    SX126X_WRITE_REGISTER = 0x0D,
    SX126X_READ_REGISTER  = 0x1D,
    SX126X_WRITE_BUFFER   = 0x0E,
    SX126X_READ_BUFFER    = 0x1E,
    // DIO and IRQ Control Functions
    SX126X_SET_DIO_IRQ_PARAMS         = 0x08,
    SX126X_GET_IRQ_STATUS             = 0x12,
    SX126X_CLR_IRQ_STATUS             = 0x02,
    SX126X_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
    SX126X_SET_DIO3_AS_TCXO_CTRL      = 0x97,
    // RF Modulation and Packet-Related Functions
    SX126X_SET_RF_FREQUENCY          = 0x86,
    SX126X_SET_PKT_TYPE              = 0x8A,
    SX126X_GET_PKT_TYPE              = 0x11,
    SX126X_SET_TX_PARAMS             = 0x8E,
    SX126X_SET_MODULATION_PARAMS     = 0x8B,
    SX126X_SET_PKT_PARAMS            = 0x8C,
    SX126X_SET_CAD_PARAMS            = 0x88,
    SX126X_SET_BUFFER_BASE_ADDRESS   = 0x8F,
    SX126X_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,
    // Communication Status Information
    SX126X_GET_STATUS           = 0xC0,
    SX126X_GET_RX_BUFFER_STATUS = 0x13,
    SX126X_GET_PKT_STATUS       = 0x14,
    SX126X_GET_RSSI_INST        = 0x15,
    SX126X_GET_STATS            = 0x10,
    SX126X_RESET_STATS          = 0x00,
    // Miscellaneous
    SX126X_GET_DEVICE_ERRORS = 0x17,
    SX126X_CLR_DEVICE_ERRORS = 0x07,
} sx126x_commands_t;

/**
 * Commands Interface buffer sizes
 */
typedef enum sx126x_commands_size_e
{
    // Operational Modes Functions
    SX126X_SIZE_SET_SLEEP                  = 2,
    SX126X_SIZE_SET_STANDBY                = 2,
    SX126X_SIZE_SET_FS                     = 1,
    SX126X_SIZE_SET_TX                     = 4,
    SX126X_SIZE_SET_RX                     = 4,
    SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE = 2,
    SX126X_SIZE_SET_RX_DUTY_CYCLE          = 7,
    SX126X_SIZE_SET_CAD                    = 1,
    SX126X_SIZE_SET_TX_CONTINUOUS_WAVE     = 1,
    SX126X_SIZE_SET_TX_INFINITE_PREAMBLE   = 1,
    SX126X_SIZE_SET_REGULATOR_MODE         = 2,
    SX126X_SIZE_CALIBRATE                  = 2,
    SX126X_SIZE_CALIBRATE_IMAGE            = 3,
    SX126X_SIZE_SET_PA_CFG                 = 5,
    SX126X_SIZE_SET_RX_TX_FALLBACK_MODE    = 2,
    // Registers and buffer Access
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_REGISTER = 3,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_REGISTER = 4,
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_BUFFER = 2,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_BUFFER = 3,
    // DIO and IRQ Control Functions
    SX126X_SIZE_SET_DIO_IRQ_PARAMS         = 9,
    SX126X_SIZE_GET_IRQ_STATUS             = 2,
    SX126X_SIZE_CLR_IRQ_STATUS             = 3,
    SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL = 2,
    SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL      = 5,
    // RF Modulation and Packet-Related Functions
    SX126X_SIZE_SET_RF_FREQUENCY           = 5,
    SX126X_SIZE_SET_PKT_TYPE               = 2,
    SX126X_SIZE_GET_PKT_TYPE               = 2,
    SX126X_SIZE_SET_TX_PARAMS              = 3,
    SX126X_SIZE_SET_MODULATION_PARAMS_GFSK = 9,
    SX126X_SIZE_SET_MODULATION_PARAMS_LORA = 5,
    SX126X_SIZE_SET_PKT_PARAMS_GFSK        = 10,
    SX126X_SIZE_SET_PKT_PARAMS_LORA        = 7,
    SX126X_SIZE_SET_CAD_PARAMS             = 8,
    SX126X_SIZE_SET_BUFFER_BASE_ADDRESS    = 3,
    SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT  = 2,
    // Communication Status Information
    SX126X_SIZE_GET_STATUS           = 1,
    SX126X_SIZE_GET_RX_BUFFER_STATUS = 2,
    SX126X_SIZE_GET_PKT_STATUS       = 2,
    SX126X_SIZE_GET_RSSI_INST        = 2,
    SX126X_SIZE_GET_STATS            = 2,
    SX126X_SIZE_RESET_STATS          = 7,
    // Miscellaneous
    SX126X_SIZE_GET_DEVICE_ERRORS = 2,
    SX126X_SIZE_CLR_DEVICE_ERRORS = 3,
    SX126X_SIZE_MAX_BUFFER        = 255,
    SX126X_SIZE_DUMMY_BYTE        = 1,
} sx126x_commands_size_t;

// typedef struct sx126x_pa_cfg_params_s
// {
//     uint8_t pa_duty_cycle;
//     uint8_t hp_max;
//     uint8_t device_sel;
//     uint8_t pa_lut;
// } sx126x_pa_cfg_params_t;

typedef struct smtc_shield_sx126x_pa_pwr_cfg_s
{
    int8_t                 power;
    sx126x_pa_cfg_params_t pa_config;
} smtc_shield_sx126x_pa_pwr_cfg_t;

inline static uint8_t apps_common_compute_lora_ldro( const sx126x_lora_sf_t sf, const sx126x_lora_bw_t bw )
{
    switch( bw )
    {
    case SX126X_LORA_BW_500:
        return 0;

    case SX126X_LORA_BW_250:
        if( sf == SX126X_LORA_SF12 )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_125:
        if( ( sf == SX126X_LORA_SF12 ) || ( sf == SX126X_LORA_SF11 ) )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_062:
        if( ( sf == SX126X_LORA_SF12 ) || ( sf == SX126X_LORA_SF11 ) || ( sf == SX126X_LORA_SF10 ) )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_041:
        if( ( sf == SX126X_LORA_SF12 ) || ( sf == SX126X_LORA_SF11 ) || ( sf == SX126X_LORA_SF10 ) ||
            ( sf == SX126X_LORA_SF9 ) )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_031:
    case SX126X_LORA_BW_020:
    case SX126X_LORA_BW_015:
    case SX126X_LORA_BW_010:
    case SX126X_LORA_BW_007:
        return 1;

    default:
        return 0;
    }
}

static const smtc_shield_sx126x_pa_pwr_cfg_t smtc_shield_sx1262mb1cas_pa_pwr_cfg_table[SMTC_SHIELD_SX1262_MAX_PWR - SMTC_SHIELD_SX1262_MIN_PWR + 1] = {
    {   // Expected output power = -9dBm
        .power = 3,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -8dBm
        .power = 4,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = -7dBm
        .power = 7,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = -6dBm
        .power = 7,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = -5dBm
        .power = 9,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x04,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = -4dBm
        .power = 10,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = -3dBm
        .power = 8,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x03,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = -2dBm
        .power = 12,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = -1dBm
        .power = 15,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 0dBm
        .power = 14,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 1dBm
        .power = 20,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 2dBm
        .power = 19,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x02,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 3dBm
        .power = 20,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 4dBm
        .power = 21,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 5dBm
        .power = 18,
        .pa_config = {
            .hp_max        = 0x02,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 6dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 7dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 8dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x02,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 9dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x03,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 10dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x01,
            .pa_duty_cycle = 0x04,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 11dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x02,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 12dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x02,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 13dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x02,
            .pa_duty_cycle = 0x02,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 14dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x02,
            .pa_duty_cycle = 0x03,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 15dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x03,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 16dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x03,
            .pa_duty_cycle = 0x02,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 17dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x05,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 18dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x05,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 19dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x04,
            .pa_duty_cycle = 0x04,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 20dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x06,
            .pa_duty_cycle = 0x03,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 21dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x07,
            .pa_duty_cycle = 0x03,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
    {        // Expected output power = 22dBm
        .power = 22,
        .pa_config = {
            .hp_max        = 0x07,
            .pa_duty_cycle = 0x04,
            .device_sel    = 0x00,
            .pa_lut        = 0x01,
        },
    },
};

/**
 * @brief Function pointer to abstract PA and power configuration getter
 */
typedef const smtc_shield_sx126x_pa_pwr_cfg_t* ( *smtc_shield_sx126x_get_pa_pwr_cfg_f )(
    const uint32_t rf_freq_in_hz, int8_t expected_output_pwr_in_dbm );
    
/**
 * @brief SX126x shield function pointer structure definition
 */
typedef struct smtc_shield_sx126x_s
{
    smtc_shield_sx126x_get_pa_pwr_cfg_f           get_pa_pwr_cfg;
} smtc_shield_sx126x_t;

const smtc_shield_sx126x_pa_pwr_cfg_t* smtc_shield_sx1262mb1cas_get_pa_pwr_cfg(
    const uint32_t rf_freq_in_hz, const int8_t expected_output_pwr_in_dbm );

/**
 * @brief Initialization of sx126x
 */
void apps_common_sx126x_init( const void* context );

/**
 * @brief Initialize the radio part of sx126x
 */
void apps_common_sx126x_radio_init( const void* context );

#endif