#ifndef ESP_RMT_INTERRUPT_HANDLER_HPP
#define ESP_RMT_INTERRUPT_HANDLER_HPP

#include "../cadmium/simulation/rt_clock/interrupt_handler.hpp"
#include "RMT_rx_driver.hpp"

namespace cadmium::comms{
    class RMTinterruptHandler : public InterruptHandler<uint64_t> {

        public:
        bool ISRcb() {
            return (xQueueReceive(recieve_queue, &rx_data, pdMS_TO_TICKS(10)) == pdPASS);
        }

        uint64_t decodeISR() {
            return parse_data_frame(rx_data.received_symbols, rx_data.num_symbols);
        }

    };   
}

#endif