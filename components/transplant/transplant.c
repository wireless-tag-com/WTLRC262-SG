#include "stdio.h"
#include "stdlib.h"
#include "sx126x_hal.h"
#include "sx126x_regs.h"
#include "transplant.h"
#include "apps_configuration.h"

#include "esp_log.h"

#define TAG "TRANSPLANT"

static sx126x_mod_params_lora_t lora_mod_params = {
    .sf = LORA_SPREADING_FACTOR,
    .bw = LORA_BANDWIDTH,
    .cr = LORA_CODING_RATE,
    .ldro = 0,
};

const sx126x_pkt_params_lora_t lora_pkt_params = {
    .preamble_len_in_symb = LORA_PREAMBLE_LENGTH,
    .header_type = LORA_PKT_LEN_MODE,
    .pld_len_in_bytes = PAYLOAD_LENGTH,
    .crc_is_on = LORA_CRC,
    .invert_iq_is_on = LORA_IQ,
};

extern volatile bool irq_fired_flag;
smtc_shield_sx126x_t shield = {.get_pa_pwr_cfg = smtc_shield_sx1262mb1cas_get_pa_pwr_cfg};

sx126x_status_t sx126x_set_lora_sync_word(const void *context, const uint8_t sync_word);
sx126x_status_t sx126x_set_lora_pkt_params(const void *context, const sx126x_pkt_params_lora_t *params);
sx126x_status_t sx126x_set_lora_mod_params(const void *context, const sx126x_mod_params_lora_t *params);

sx126x_status_t sx126x_add_registers_to_retention_list(const void *context, const uint16_t *register_addr, uint8_t register_nb);
uint32_t get_time_on_air_in_ms(void);
static inline const smtc_shield_sx126x_pa_pwr_cfg_t *smtc_shield_sx126x_get_pa_pwr_cfg(
    uint8_t *shield, const uint32_t rf_freq_in_hz, int8_t expected_output_pwr_in_dbm);

extern void on_tx_done(void);
extern void on_tx_done(void);
extern void on_rx_done(void);
extern void apps_common_sx126x_receive(const void *context, uint8_t *buffer, uint8_t *size, uint8_t max_size);



void apps_common_sx126x_init(const void *context)
{
    sx126x_reset((void *)context);

    sx126x_init_retention_list((void *)context);

    sx126x_set_reg_mode(context, SX126X_REG_MODE_DCDC);

    sx126x_set_dio2_as_rf_sw_ctrl(context, true);
}

void apps_common_sx126x_radio_init(const void *context)
{
    const smtc_shield_sx126x_pa_pwr_cfg_t *pa_pwr_cfg =
        smtc_shield_sx126x_get_pa_pwr_cfg((uint8_t *)&shield, RF_FREQ_IN_HZ, TX_OUTPUT_POWER_DBM);

    if (pa_pwr_cfg == NULL)
    {
        printf("Invalid target frequency or power level\n");
        while (true)
        {
        }
    }
    sx126x_set_standby(context, SX126X_STANDBY_CFG_RC);
    sx126x_set_pkt_type(context, PACKET_TYPE);
    sx126x_set_rf_freq(context, RF_FREQ_IN_HZ);

    sx126x_set_pa_cfg(context, &(pa_pwr_cfg->pa_config));
    sx126x_set_tx_params(context, pa_pwr_cfg->power, PA_RAMP_TIME);

    sx126x_set_rx_tx_fallback_mode(context, FALLBACK_MODE);
    sx126x_cfg_rx_boosted(context, ENABLE_RX_BOOST_MODE);

    if (PACKET_TYPE == SX126X_PKT_TYPE_LORA)
    {
        lora_mod_params.ldro = apps_common_compute_lora_ldro(LORA_SPREADING_FACTOR, LORA_BANDWIDTH);
        sx126x_set_lora_mod_params(context, &lora_mod_params);
        sx126x_set_lora_pkt_params(context, &lora_pkt_params);
        sx126x_set_lora_sync_word(context, LORA_SYNCWORD);
    }
}

void apps_common_sx126x_irq_process(const void *context)
{
    // static int cnt = 0;
    struct spi_context *spi_context_t = (struct spi_context *)context;

    if (irq_fired_flag == true)
    {
        irq_fired_flag = false;

        sx126x_irq_mask_t irq_regs;
        sx126x_get_and_clear_irq_status(context, &irq_regs);

        if ((irq_regs & SX126X_IRQ_TX_DONE) == SX126X_IRQ_TX_DONE)
        {
            ESP_LOGI(TAG, "Tx done");
            on_tx_done();
        }

        if ((irq_regs & SX126X_IRQ_RX_DONE) == SX126X_IRQ_RX_DONE)
        {
            ESP_LOGI(TAG, "Rx done");
            sx126x_handle_rx_done(spi_context_t);
            if (PACKET_TYPE == SX126X_PKT_TYPE_GFSK)
            {
                sx126x_pkt_status_gfsk_t pkt_status;
                sx126x_get_gfsk_pkt_status(context, &pkt_status);

                if (pkt_status.rx_status.crc_error == true)
                {
                    ESP_LOGE(TAG, "CRC error from packet status");
                }
                else if (pkt_status.rx_status.adrs_error == true)
                {
                    ESP_LOGE(TAG, "Address error from packet status");
                }
                else if (pkt_status.rx_status.length_error == true)
                {
                    ESP_LOGE(TAG, "Length error from packet status");
                }
                else
                {
                    on_rx_done( );
                }
            }
            else
            {
                on_rx_done();
                printf("on_rx_done()\n");
            }
        }

        if ((irq_regs & SX126X_IRQ_PREAMBLE_DETECTED) == SX126X_IRQ_PREAMBLE_DETECTED)
        {
            ESP_LOGI(TAG, "Preamble detected");
        }

        if ((irq_regs & SX126X_IRQ_SYNC_WORD_VALID) == SX126X_IRQ_SYNC_WORD_VALID)
        {
            ESP_LOGI(TAG, "Syncword valid");
        }

        if ((irq_regs & SX126X_IRQ_HEADER_VALID) == SX126X_IRQ_HEADER_VALID)
        {
            ESP_LOGI(TAG, "Header valid");
        }

        if ((irq_regs & SX126X_IRQ_HEADER_ERROR) == SX126X_IRQ_HEADER_ERROR)
        {
            ESP_LOGE(TAG, "Header error");
        }

        if ((irq_regs & SX126X_IRQ_CRC_ERROR) == SX126X_IRQ_CRC_ERROR)
        {
            ESP_LOGE(TAG, "CRC error");
        }

        if ((irq_regs & SX126X_IRQ_CAD_DONE) == SX126X_IRQ_CAD_DONE)
        {
            ESP_LOGI(TAG, "CAD done");
            if ((irq_regs & SX126X_IRQ_CAD_DETECTED) == SX126X_IRQ_CAD_DETECTED)
            {
                ESP_LOGI(TAG, "Channel activity detected");
            }
            else
            {
                ESP_LOGI(TAG, "No channel activity detected");
            }
        }

        if ((irq_regs & SX126X_IRQ_TIMEOUT) == SX126X_IRQ_TIMEOUT)
        {
            ESP_LOGW(TAG, "Rx timeout");
        }

        if ((irq_regs & SX126X_IRQ_LR_FHSS_HOP) == SX126X_IRQ_LR_FHSS_HOP)
        {
            ESP_LOGI(TAG, "FHSS hop done");
        }
    }

    // printf("cnt = %d\n", cnt++);
}

uint32_t get_time_on_air_in_ms(void)
{
    switch (PACKET_TYPE)
    {
        case SX126X_PKT_TYPE_LORA:
        {
            return sx126x_get_lora_time_on_air_in_ms(&lora_pkt_params, &lora_mod_params);
        }
        case SX126X_PKT_TYPE_LR_FHSS:
        default:
        {
            return 0;
        }
    }
}

const smtc_shield_sx126x_pa_pwr_cfg_t *smtc_shield_sx1262mb1cas_get_pa_pwr_cfg(
    const uint32_t rf_freq_in_hz, const int8_t expected_output_pwr_in_dbm)
{
    if ((SMTC_SHIELD_SX126X_FREQ_MIN <= rf_freq_in_hz) && (rf_freq_in_hz <= SMTC_SHIELD_SX126X_FREQ_MAX))
    {
        if ((SMTC_SHIELD_SX1262_MIN_PWR <= expected_output_pwr_in_dbm) &&
            (expected_output_pwr_in_dbm <= SMTC_SHIELD_SX1262_MAX_PWR))
        {
            return &(
                smtc_shield_sx1262mb1cas_pa_pwr_cfg_table[expected_output_pwr_in_dbm - SMTC_SHIELD_SX1262_MIN_PWR]);
        }
    }

    return NULL;
}

static inline const smtc_shield_sx126x_pa_pwr_cfg_t *smtc_shield_sx126x_get_pa_pwr_cfg(
    uint8_t *shield, const uint32_t rf_freq_in_hz, int8_t expected_output_pwr_in_dbm)
{
    return smtc_shield_sx1262mb1cas_get_pa_pwr_cfg(rf_freq_in_hz, expected_output_pwr_in_dbm);
}
