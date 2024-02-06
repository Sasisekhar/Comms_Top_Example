#ifndef ME_RX_HPP
#define ME_RX_HPP

#include <iostream>
#include "cadmium/modeling/devs/atomic.hpp"
#include "dll_frame.hpp"
#include "comms_defines.hpp"

#ifdef RT_ESP32
#include "drivers/RMT_rx_driver.hpp"
#endif

namespace cadmium::comms {

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
        public:

        Port<dll_frame> out;
        Port<uint64_t> in;

        //Constructor
        ME_rx(const std::string id, uint32_t _rxpin, uint32_t _tickres): Atomic<ME_rxState>(id, ME_rxState()){
            out = addOutPort<dll_frame> ("out");
            in = addInPort<uint64_t> ("in");

#ifdef RT_ESP32
            initRmtRecieve((gpio_num_t)_rxpin, _tickres);
#endif
        }

        // internal transition
        void internalTransition(ME_rxState& state) const override {
            state.transmit = false;
            state.sigma = std::numeric_limits<double>::infinity();
        }

        // external transition
        void externalTransition(ME_rxState& state, double e) const override {

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