#ifndef ME_RX_HPP
#define ME_RX_HPP

#include <iostream>
#include "cadmium/modeling/devs/atomic.hpp"
#include "dll_frame.hpp"
#include "comms_defines.hpp"

#include "cadmium/simulation/rt_clock/interrupt_handler.hpp"

#ifdef RT_ESP32
#include <driver/gpio.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
// #include "ets_sys.h"
#endif

namespace cadmium::comms {

#ifdef RT_ESP32
    //per proper cpp, this should be static. !TODO follow cpp norms
    rmt_symbol_word_t raw_symbols[64]; // !TODO change to 35
#endif


    struct ME_rxState {
        uint64_t in_data;
        dll_frame model_output;
        bool transmit;
        double sigma;
        double deadline;

        /**
         * Processor state constructor. By default, the processor is idling.
         * 
         */
        explicit ME_rxState(): in_data(0), model_output(), transmit(false), sigma(std::numeric_limits<double>::infinity()), deadline(1.0){
        }
    };

#ifndef NO_LOGGING
    std::ostream& operator<<(std::ostream &out, const ME_rxState& state) {
        out << "in_data: " << std::hex << state.in_data;
        return out;
    }
#endif

    class ME_rx : public Atomic<ME_rxState> {
        private:

#ifdef RT_ESP32
        const char* TAG = "[ME_RX]"; //for logging
        uint32_t TICK_RESOLUTION = 80 * 1000 * 1000;
        rmt_receive_config_t rx_config;
        rmt_channel_handle_t rx_channel;

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
            cadmium::interrupt::recieve_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
            assert(cadmium::interrupt::recieve_queue);

            rmt_rx_event_callbacks_t cbs = {
                .on_recv_done = rmt_rx_done_callback,
            };
            
            ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, &cadmium::interrupt::recieve_queue));

            ESP_LOGI(TAG, "Enabling PHY RX channel");
            ESP_ERROR_CHECK(rmt_enable(rx_channel));
        }

        static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
            BaseType_t high_task_wakeup = pdFALSE;
            QueueHandle_t receive_queue = (QueueHandle_t)cadmium::interrupt::recieve_queue;
            xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
            return high_task_wakeup == pdTRUE;
        }
#endif

        public:

        Port<dll_frame> out;
        Port<uint64_t> in;

#ifdef RT_ESP32
        gpio_num_t rxpin;
#else
        using gpio_num_t = uint32_t;
#endif

        //Constructor
        ME_rx(const std::string id, gpio_num_t _rxpin, uint32_t _tickres): Atomic<ME_rxState>(id, ME_rxState()){
            out = addOutPort<dll_frame> ("out");
            in = addInPort<uint64_t> ("in");

#ifdef RT_ESP32
            rxpin = _rxpin;
            TICK_RESOLUTION = _tickres;
            rx_config.signal_range_min_ns = 90;
            rx_config.signal_range_max_ns = 350;
            config_channel_encoders();
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &rx_config));
#endif
        }

        // internal transition
        void internalTransition(ME_rxState& state) const override {
            state.transmit = false;
            state.sigma = std::numeric_limits<double>::infinity();
        }

        // external transition
        void externalTransition(ME_rxState& state, double e) const override {
#ifdef RT_ESP32
            // start receive again
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &rx_config));
#endif
            if(!in->empty()){
                for( const auto x : in->getBag()){
                    state.in_data = x;
                }
            }

            state.model_output.data                 = (uint32_t) (PAYLOAD_MASK & state.in_data);
            state.model_output.frame_num            = (uint8_t)  ((FRAME_MASK & state.in_data) >> PAYLOAD_LEN);
            state.model_output.total_frames         = (uint8_t)  ((TOTAL_FRAMES_MASK & state.in_data) >> (PAYLOAD_LEN + FRAME_NUM_LEN));
            state.model_output.datalen_frame_select = (uint8_t)  ((SELECT_MASK & state.in_data) >> (PAYLOAD_LEN + FRAME_NUM_LEN + FRAME_NUM_LEN));
            state.model_output.checksum             = (uint8_t)  ((CHECKSUM_MASK & state.in_data) >> (PAYLOAD_LEN + FRAME_NUM_LEN + FRAME_NUM_LEN + SELECT_LEN));

            state.sigma = 0;
            state.transmit = true;

        }
        
        
        // output function
        void output(const ME_rxState& state) const override {
            if(state.transmit){
                out->addMessage(state.model_output);
            }
        }

        // time_advance function
        [[nodiscard]] double timeAdvance(const ME_rxState& state) const override {     
            //   return std::numeric_limits<double>::infinity();
            return state.sigma;
        }
        
        void confluentTransition(ME_rxState& s, double e) const {
            this->externalTransition(s, e);
            this->internalTransition(s);
        }

    };
}

#endif