#ifndef INTERRUPT_HANDLER_HPP
#define INTERRUPT_HANDLER_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"

namespace cadmium::interrupt {
    QueueHandle_t recieve_queue;
    rmt_rx_done_event_data_t rx_data;

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

        return(dataDecoded);
    }
}

#endif