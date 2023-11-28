#ifndef ME_TX_HPP
#define ME_TX_HPP

#include <iostream>

#include "cadmium/modeling/devs/atomic.hpp"
#include "dll_frame.hpp"
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
    
    struct ME_txState {
        dll_frame in_data;
        uint64_t out_data;
        double sigma;
        double deadline;

        /**
         * Processor state constructor. By default, the processor is idling.
         * 
         */
        explicit ME_txState(): in_data(), out_data(0), sigma(std::numeric_limits<double>::infinity()), deadline(1.0){
        }
    };

#ifndef NO_LOGGING
    std::ostream& operator<<(std::ostream &out, const ME_txState& state) {
        out << "HW_out: " << std::hex << state.out_data;
        return out;
    }
#endif

    class ME_tx : public Atomic<ME_txState> {
        private:
        const char* TAG = "[ME_TX]"; //for logging
        uint32_t TICK_RESOLUTION = 80 * 1000 * 1000;
        rmt_transmit_config_t tx_config;
        rmt_channel_handle_t tx_channel;
        rmt_encoder_handle_t manchester_encoder;

        //Helper functions
        void config_channel_encoders() {

            ESP_LOGI(TAG, "Creating PHY TX channel");
            rmt_tx_channel_config_t tx_chan_config = {
                .gpio_num = txpin,
                .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
                .resolution_hz = TICK_RESOLUTION,
                .mem_block_symbols = 72,
                .trans_queue_depth = 3,
                .intr_priority = 0,
            };

            tx_chan_config.flags.invert_out = false;
            tx_chan_config.flags.with_dma = false;
            tx_chan_config.flags.io_loop_back = false;
            tx_chan_config.flags.io_od_mode = false;
            ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_channel));

            ESP_LOGI(TAG, "Creating manchester encoder");

            manchester_encoder_config_t manchester_encoder_config = {
                .resolution = TICK_RESOLUTION,
            };
            ESP_ERROR_CHECK(rmt_new_manchester_encoder(&manchester_encoder_config, &manchester_encoder));

            ESP_LOGI(TAG, "Enabling PHY TX channel");
            ESP_ERROR_CHECK(rmt_enable(tx_channel));
        }

        public:

        Port<dll_frame> in;

        gpio_num_t txpin;

        //Constructor
        ME_tx(const std::string id, gpio_num_t _txpin, uint32_t _tickres): Atomic<ME_txState> (id, ME_txState()) {
            in = addInPort<dll_frame> ("in");
            txpin = _txpin;
            TICK_RESOLUTION = _tickres;
            tx_config.loop_count = 0;
            tx_config.flags.eot_level = false; //important! this value gets corrupted when numerous atomics are placed
            config_channel_encoders();
        }

        // internal transition
        void internalTransition(ME_txState& state) const override {
            state.sigma = std::numeric_limits<double>::infinity();
        }

        // external transition
        void externalTransition(ME_txState& state, double e) const override {
            if(!in->empty()){
                for( const auto x : in->getBag()){
                    state.in_data = x;
                }
            }

            uint64_t out = 0;
            for(int i = (32 + 6 + 8) - 1; i >= 0; i--) { //iterator starting value is sizeof(data + framenum + checksum)
                uint64_t tmp = 0;
                if(i < 32) {
                    tmp = ((state.in_data.data & ((0x00000001) << i)) >> i);
                } else if(i >= 32 && i < 38) {
                    tmp = ((state.in_data.frame_num & ((0x01) << (i - 32))) >> (i - 32));
                } else {
                    tmp = ((state.in_data.checksum & ((0x01) << (i - 38))) >> (i - 38));
                }

                out |= tmp << i;
            }
            state.out_data = out;

            state.sigma = 0;
            // ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
        }
        
        
        // output function
        void output(const ME_txState& state) const override {
#ifdef NO_LOGGING
            ESP_LOGI(TAG, "Transmitted: 0x%llx", state.out_data);
#endif

            ESP_ERROR_CHECK(rmt_transmit(tx_channel, manchester_encoder, &state.out_data, sizeof(uint64_t), &tx_config));
        }

        // time_advance function
        [[nodiscard]] double timeAdvance(const ME_txState& state) const override {     
            //   return std::numeric_limits<double>::infinity();
            return state.sigma;
        }

    };
}

#endif