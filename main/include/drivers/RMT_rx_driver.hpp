#ifndef RMT_RX_DRIVER_HPP
#define RMT_RX_DRIVER_HPP

#include <driver/gpio.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"

    namespace cadmium::comms {
    rmt_symbol_word_t raw_symbols[64];
    const char* TAG = "[ME_RX]"; //for logging
    uint32_t TICK_RESOLUTION = 80 * 1000 * 1000;
    rmt_receive_config_t rx_config;
    rmt_channel_handle_t rx_channel;
    gpio_num_t rxpin;
    QueueHandle_t recieve_queue;
    rmt_rx_done_event_data_t rx_data;

    //ISR
    bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
        BaseType_t high_task_wakeup = pdFALSE;
        QueueHandle_t receive_queue = (QueueHandle_t)recieve_queue;
        xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
        return high_task_wakeup == pdTRUE;
    }

    //Helper functions
    void config_channel_encoders() {
        
        ESP_LOGI(TAG, "Creating PHY TX channel");
        rmt_rx_channel_config_t rx_channel_cfg = {
            .gpio_num = rxpin,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = TICK_RESOLUTION,
            .mem_block_symbols = 72, // amount of RMT symbols that the channel can store at a time
        };
        ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));

        ESP_LOGI(TAG, "Registering Queue and Rx done callback");
        recieve_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
        assert(recieve_queue);

        rmt_rx_event_callbacks_t cbs = {
            .on_recv_done = rmt_rx_done_callback,
        };
        
        ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, &recieve_queue));

        ESP_LOGI(TAG, "Enabling PHY RX channel");
        ESP_ERROR_CHECK(rmt_enable(rx_channel));
    }

    uint64_t parse_data_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num) {

        int total_time = 0;
        for (size_t i = 0; i < symbol_num; i++) {
            total_time += rmt_nec_symbols[i].duration0 + rmt_nec_symbols[i].duration1;
        }

        int ts = 18;
        int c = 0;
        int i = 0;
        bool data[72] = { false };
        int dataIndex = 0;

        while(c < total_time) {
            c += rmt_nec_symbols[i].duration0;

            if(c > ts) {
                if(rmt_nec_symbols[i].level0 == 1)
                    data[dataIndex++] = false;
                else
                    data[dataIndex++] = true;

                ts += 16;
            }

            c += rmt_nec_symbols[i].duration1;

            if(c > ts) {
                
                if(rmt_nec_symbols[i].level1 == 0)
                    data[dataIndex++] = true;
                else
                    data[dataIndex++] = false;

                ts += 16;
            }

            i++;
        }
        uint64_t dataDecoded = 0;

        for(int i = 1; i < dataIndex - 4; i++) {
            uint64_t tmp = (data[i])? 1 : 0;
            dataDecoded |= tmp << (i - 1);
        }

        // start receive again
        ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &rx_config));
        return(dataDecoded);
    }

    void initRmtRecieve(gpio_num_t _rxpin, uint32_t _tickres) {
        rxpin = _rxpin;
        TICK_RESOLUTION = _tickres;
        rx_config.signal_range_min_ns = 90;
        rx_config.signal_range_max_ns = 350;
        config_channel_encoders();
        ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &rx_config));
    }
}
#endif