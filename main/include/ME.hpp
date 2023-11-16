#ifndef ME_HPP
#define ME_HPP

#include <iostream>

#include "cadmium/modeling/devs/atomic.hpp"
#include <driver/gpio.h>
#include "esp_system.h"
#include "esp_console.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "drivers/manchester_encoder.h"

namespace cadmium::comms {
    const char* TAG = "ME Atomic"; //for logging

    //per proper cpp, this should be static. !TODO follow cpp norms
    rmt_symbol_word_t raw_symbols[64]; // !TODO change to 35
    QueueHandle_t receive_queue;

    struct MEState {
        uint32_t inData;
        uint32_t outData;
        double sigma;
        double deadline;

        /**
         * Processor state constructor. By default, the processor is idling.
         * 
         */
        explicit MEState(): inData(0), outData(0), sigma(1), deadline(1.0){
        }
    };

    std::ostream& operator<<(std::ostream &out, const MEState& state) {
        out << "Rx: " << std::hex << state.inData << ",Tx: " << std::hex << state.outData;
        return out;
    }

    class ME : public Atomic<MEState> {
        private:
        uint32_t TICK_RESOLUTION = 80 * 1000 * 1000;
        rmt_receive_config_t rx_config;
        rmt_transmit_config_t tx_config;
        rmt_channel_handle_t rx_channel;
        rmt_channel_handle_t tx_channel;
        rmt_encoder_handle_t manchester_encoder;

        //Helper functions
        void config_channel_encoders() {
            
            ESP_LOGI(TAG, "Creating PHY RX channel");
            rmt_rx_channel_config_t rx_channel_cfg = {
                .gpio_num = rxpin,
                .clk_src = RMT_CLK_SRC_DEFAULT,
                .resolution_hz = TICK_RESOLUTION,
                .mem_block_symbols = 48, // amount of RMT symbols that the channel can store at a time
            };
            ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));

            ESP_LOGI(TAG, "Creating PHY TX channel");
            rmt_tx_channel_config_t tx_chan_config = {
                .gpio_num = txpin,
                .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
                .resolution_hz = TICK_RESOLUTION,
                .mem_block_symbols = 128,
                .trans_queue_depth = 3,
                .intr_priority = 0,
            };

            tx_chan_config.flags.invert_out = false;
            tx_chan_config.flags.with_dma = false;
            tx_chan_config.flags.io_loop_back = false;
            tx_chan_config.flags.io_od_mode = false;
            ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_channel));

            ESP_LOGI(TAG, "Registering Queue and Rx done callback");
            receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
            assert(receive_queue);

            rmt_rx_event_callbacks_t cbs = {
                .on_recv_done = rmt_rx_done_callback,
            };
            
            ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));

            ESP_LOGI(TAG, "Creating encoders");

            manchester_encoder_config_t manchester_encoder_config = {
                .resolution = TICK_RESOLUTION,
            };
            ESP_ERROR_CHECK(rmt_new_manchester_encoder(&manchester_encoder_config, &manchester_encoder));

            ESP_LOGI(TAG, "Enabling RMT channel");
            ESP_ERROR_CHECK(rmt_enable(tx_channel));
            ESP_ERROR_CHECK(rmt_enable(rx_channel));
        }

        static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
            BaseType_t high_task_wakeup = pdFALSE;
            QueueHandle_t receive_queue = (QueueHandle_t)user_data;
            // send the received RMT symbols to the parser task
            xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
            return high_task_wakeup == pdTRUE;
        }

        static uint32_t parse_data_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num) {

            int ts = 18;
            int c = 0;
            int i = 0;
            bool data[36] = { false };
            int dataIndex = 0;

            while(c < 567) {
                c += rmt_nec_symbols[i].duration0;

                if(c > ts) {
                    // printf("Sampled, Duration 0, i: %d\r\n", i);
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

            // printf("0b");
            uint32_t dataDecoded = 0;
            for(int i = 32; i > 0; i--) {
                // printf("%d", (data[i])? 1 : 0);
                dataDecoded |= ((data[i])? 1 << (i - 1) : 0 << (i - 1));
            }

            ESP_LOGI("parse_data_frame", "Recieved frame: 0x%lx", dataDecoded);


            return(dataDecoded);
        }

        public:

        Port<uint32_t> in;
        Port<uint32_t> out;

        gpio_num_t txpin;
        gpio_num_t rxpin;

        //Constructor
        ME(const std::string id, gpio_num_t _txpin, gpio_num_t _rxpin, uint32_t _tickres): Atomic<MEState> (id, MEState()) {
            in = addInPort<uint32_t> ("in");
            out = addOutPort<uint32_t> ("out");

            txpin = _txpin;
            rxpin = _rxpin;

            // gpio_reset_pin(txpin);
            // gpio_set_direction(txpin, GPIO_MODE_OUTPUT);
            // gpio_set_level(txpin, false);

            TICK_RESOLUTION = _tickres;

            rx_config.signal_range_min_ns = 90;
            rx_config.signal_range_max_ns = 350;

            tx_config.loop_count = 0;
            tx_config.flags.eot_level = false; //important! this value gets corrupted when numerous atomics are placed
            config_channel_encoders();
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &rx_config));
        }

        // internal transition
        void internalTransition(MEState& state) const override {
            rmt_rx_done_event_data_t rx_data;
            if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(500)) == pdPASS) {

                // parse the receive symbols and print the result
                state.inData = parse_data_frame(rx_data.received_symbols, rx_data.num_symbols);

                // start receive again
                ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &rx_config));
            }
        }

        // external transition
        void externalTransition(MEState& state, double e) const override {
            if(!in->empty()){
                for( const auto x : in->getBag()){
                    state.outData = x;
                }
            }

            // ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
            ESP_LOGI(TAG, "Transmitted: 0x%lx", state.outData);
            ESP_ERROR_CHECK(rmt_transmit(tx_channel, manchester_encoder, &state.outData, sizeof(uint32_t), &tx_config));
            
        }
        
        
        // output function
        void output(const MEState& state) const override {
            out->addMessage(state.inData);
        }

        // time_advance function
        [[nodiscard]] double timeAdvance(const MEState& state) const override {     
            //   return std::numeric_limits<double>::infinity();
            return state.sigma;
        }

    };
}

#endif