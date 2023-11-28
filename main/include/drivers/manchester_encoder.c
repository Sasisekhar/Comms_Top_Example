/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_check.h"
#include "manchester_encoder.h"

static const char *TAG = "ME_DRIVER";

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_handle_t copy_encoder;
    rmt_encoder_handle_t bytes_encoder;
    uint32_t resolution;
} rmt_manchester_encoder_t;

static size_t rmt_encode_manchester_encoder(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_manchester_encoder_t *motor_encoder = __containerof(encoder, rmt_manchester_encoder_t, base);

    rmt_encoder_handle_t copy_encoder = motor_encoder->copy_encoder;
    rmt_encoder_handle_t bytes_encoder = motor_encoder->bytes_encoder;

    rmt_encode_state_t session_state = 0;
    uint64_t data = *(uint64_t *)primary_data;

    rmt_symbol_word_t leading_bit = {
        .level0 = 0,
        .duration0 = 8,
        .level1 = 1,
        .duration1 = 18,
    };

    size_t encoded_symbols = copy_encoder->encode(copy_encoder, channel, &leading_bit, sizeof(leading_bit), &session_state);

    encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &data, data_size, &session_state);

    rmt_symbol_word_t trailing_bit = {
        .level0 = 0,
        .duration0 = 8,
        .level1 = 1,
        .duration1 = 28,
    };

    encoded_symbols += copy_encoder->encode(copy_encoder, channel, &trailing_bit, sizeof(trailing_bit), &session_state);

    *ret_state = session_state;
    return encoded_symbols;
}

static esp_err_t rmt_del_manchester_encoder(rmt_encoder_t *encoder)
{
    rmt_manchester_encoder_t *man_encoder = __containerof(encoder, rmt_manchester_encoder_t, base);
    rmt_del_encoder(man_encoder->copy_encoder);
    free(man_encoder);
    return ESP_OK;
}

static esp_err_t rmt_reset_manchester_encoder(rmt_encoder_t *encoder)
{
    rmt_manchester_encoder_t *man_encoder = __containerof(encoder, rmt_manchester_encoder_t, base);
    rmt_encoder_reset(man_encoder->copy_encoder);
    return ESP_OK;
}

esp_err_t rmt_new_manchester_encoder(const manchester_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_manchester_encoder_t *man_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid arguments");
    man_encoder = calloc(1, sizeof(rmt_manchester_encoder_t));
    ESP_GOTO_ON_FALSE(man_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for stepper uniform encoder");
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &man_encoder->copy_encoder), err, TAG, "create copy encoder failed");

    man_encoder->resolution = config->resolution;
    man_encoder->base.del = rmt_del_manchester_encoder;
    man_encoder->base.encode = rmt_encode_manchester_encoder;
    man_encoder->base.reset = rmt_reset_manchester_encoder;

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .flags.msb_first = 0,
        .bit0 = {
            .level0 = 1,
            .duration0 = 8, //ns
            .level1 = 0,
            .duration1 = 8, //500ns
        },
        .bit1 = {
            .level0 = 0,
            .duration0 = 8, //500ns
            .level1 = 1,
            .duration1 = 8, //500ns
        },
    };
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &man_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");

    *ret_encoder = &(man_encoder->base);
    return ESP_OK;
err:
    if (man_encoder) {
        if (man_encoder->copy_encoder) {
            rmt_del_encoder(man_encoder->copy_encoder);
        }

        if (man_encoder->bytes_encoder) {
            rmt_del_encoder(man_encoder->bytes_encoder);
        }
        free(man_encoder);
    }
    return ret;
}