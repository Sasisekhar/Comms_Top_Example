#ifndef ME_RX_HPP
#define ME_RX_HPP

#include <iostream>
#include "cadmium/modeling/devs/atomic.hpp"
#include "dll_frame.hpp"

#ifdef RT_ESP32
#include <driver/gpio.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "ets_sys.h"
#endif

namespace cadmium::comms {

#ifdef RT_ESP32
    //per proper cpp, this should be static. !TODO follow cpp norms
    rmt_symbol_word_t raw_symbols[64]; // !TODO change to 35
    typedef struct {
        QueueHandle_t recieve_queue;
        void* class_ptr;
    } q_pkt;
    q_pkt queue_packet;
#endif


    struct ME_rxState {
        uint64_t in_data;
        dll_frame model_output;
        size_t frame_num;
        double sigma;
        double deadline;

        /**
         * Processor state constructor. By default, the processor is idling.
         * 
         */
        explicit ME_rxState(): in_data(0), model_output(), frame_num(0), sigma(0.01), deadline(1.0){
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
            queue_packet.recieve_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
            assert(queue_packet.recieve_queue);

            rmt_rx_event_callbacks_t cbs = {
                .on_recv_done = rmt_rx_done_callback,
            };
            
            ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, &queue_packet));

            ESP_LOGI(TAG, "Enabling PHY RX channel");
            ESP_ERROR_CHECK(rmt_enable(rx_channel));
        }

        static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
            BaseType_t high_task_wakeup = pdFALSE;
            q_pkt* ptr_queue_struct = (q_pkt*)user_data;
            QueueHandle_t receive_queue = (QueueHandle_t)ptr_queue_struct->recieve_queue;
            ME_rx* class_ptr = (ME_rx*)ptr_queue_struct->class_ptr;
            xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
            return high_task_wakeup == pdTRUE;
        }

        static uint64_t parse_data_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num) {

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

#ifdef NO_LOGGING
            ESP_LOGI("[ME_RX]", "Recieved frame: 0x%llx", dataDecoded);
#endif

            return(dataDecoded);
        }
#endif

        public:

        Port<dll_frame> out;

#ifdef RT_ESP32
        gpio_num_t rxpin;
#else
        using gpio_num_t = uint32_t;
        Port<uint64_t> in;
#endif

        //Constructor
        ME_rx(const std::string id, gpio_num_t _rxpin, uint32_t _tickres): Atomic<ME_rxState>(id, ME_rxState()){
            out = addOutPort<dll_frame> ("out");

#ifdef RT_ESP32
            queue_packet.class_ptr = this;
            rxpin = _rxpin;
            TICK_RESOLUTION = _tickres;
            rx_config.signal_range_min_ns = 90;
            rx_config.signal_range_max_ns = 350;
            config_channel_encoders();
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &rx_config));
#else
            in = addInPort<uint64_t> ("in");
#endif
        }

        // internal transition
        void internalTransition(ME_rxState& state) const override {
#ifdef RT_ESP32
            rmt_rx_done_event_data_t rx_data;
            if (xQueueReceive(queue_packet.recieve_queue, &rx_data, pdMS_TO_TICKS(10)) == pdPASS) {

                // ESP_LOGI("[DEBUG] DELint", "Queue filled: %d", uxQueueMessagesWaiting(receive_queue));
                // parse the receive symbols and print the result
                state.in_data = parse_data_frame(rx_data.received_symbols, rx_data.num_symbols);
                state.model_output.data         = (uint32_t) (0x00000000FFFFFFFF & state.in_data);
                state.model_output.frame_num    = (uint8_t)  (0x0000001F00000000 & state.in_data) >> 32;
                state.model_output.total_frames = (uint8_t)  (0x000003E000000000 & state.in_data) >> 37;
                state.model_output.checksum     = (uint8_t)  (0x0003FC0000000000 & state.in_data) >> 42;

                // start receive again
                ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &rx_config));
            }
#else
            state.sigma = std::numeric_limits<double>::infinity();
#endif
        }

        // external transition
        void externalTransition(ME_rxState& state, double e) const override {
#ifdef RT_ESP32
            //Nothing done here now, comeback once interrupt implemented
#else
            if(!in->empty()){
                for( const auto x : in->getBag()){
                    state.in_data = x;
                }
            }

            state.model_output.data         = (uint32_t) (0x00000000FFFFFFFF & state.in_data);
            state.model_output.frame_num    = (uint8_t)  ((0x0000001F00000000 & state.in_data) >> 32);
            state.model_output.total_frames = (uint8_t)  ((0x000003E000000000 & state.in_data) >> 37);
            state.model_output.checksum     = (uint8_t)  ((0x0003FC0000000000 & state.in_data) >> 42);

            state.sigma = 0;
#endif
        }
        
        
        // output function
        void output(const ME_rxState& state) const override {
            out->addMessage(state.model_output);
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