#ifndef DLL_HPP
#define DLL_HPP

#include "cadmium/modeling/devs/atomic.hpp"
#include "tcl_packet.hpp"
#include "dll_frame.hpp"
#include "esp_log.h"
#include <iostream>

namespace cadmium {

    struct dllState {
        tcl_packet tx_in_data;
        dll_frame tx_out_data;
        dll_frame rx_in_data;
        tcl_packet rx_out_data;
        size_t frame_num;
        double sigma;
        double deadline;

        /**
         * Processor state constructor. By default, the processor is idling.
         * 
         */
        explicit dllState(): tx_in_data(), tx_out_data(), rx_in_data(), rx_out_data(), frame_num(0), sigma(std::numeric_limits<double>::infinity()), deadline(1.0){
        }
    };

#ifndef NO_LOGGING
    /**
     * Insertion operator for GeneratorState objects. It only displays the value of sigma.
     * @param out output stream.
     * @param s state to be represented in the output stream.
     * @return output stream with sigma already inserted.
     */
    std::ostream& operator<<(std::ostream &out, const dllState& state) {
        out << "tx_in_data: " << std::hex << state.tx_in_data;
        return out;
    }
#endif

    class dll : public Atomic<dllState> {
        private:

        static uint8_t computeChecksum(dllState& state) {
            uint8_t checksum = 0;
            for(int i = 0; i < 32; i++) { //!ATTENTION hardcoded value of 32
                checksum += ((state.tx_in_data.data[state.tx_out_data.frame_num - 1] & ((0x00000001) << i)) >> i);
            }

            for(int i = 0; i < 6; i++) {
                checksum += ((state.tx_out_data.frame_num & ((0x01) << i)) >> i);
            }
            return checksum;
        }

        public:
        Port<tcl_packet> in;
        Port<dll_frame> out;

        dll(const std::string id) : Atomic<dllState>(id, dllState()) {
            in = Component::addInPort<tcl_packet>("in");
            out = Component::addOutPort<dll_frame>("out");
        }

        void internalTransition(dllState& state) const override {
            if(state.frame_num < state.tx_in_data.data.size() + 1) {
                state.tx_out_data.data = state.tx_in_data.data[state.frame_num - 1];
                state.tx_out_data.frame_num = state.frame_num;
                state.tx_out_data.checksum = computeChecksum(state);

                state.frame_num++;
            } else {
                state.sigma = std::numeric_limits<double>::infinity();
            }
        }

        // external transition
        void externalTransition(dllState& state, double e) const override {
            state.tx_in_data.data.clear();
            if(!in->empty()){
                for( const auto& x : in->getBag()){
                    state.tx_in_data = x;
                }
            }
            state.frame_num = 1;
            state.sigma = 0.02;

            if(state.frame_num < state.tx_in_data.data.size() + 1) {
                state.tx_out_data.data = state.tx_in_data.data[state.frame_num - 1];
                state.tx_out_data.frame_num = state.frame_num;
                state.tx_out_data.checksum = computeChecksum(state);
                state.frame_num++;
            } else {
                state.sigma = std::numeric_limits<double>::infinity();
            }
        }
        
        
        // output function
        void output(const dllState& state) const override {
            // ESP_LOGI("[DLL LAMBDA]", "Sent: 0x%llx", state.tx_out_data);
            out->addMessage(state.tx_out_data);
        }

        void confluentTransition(dllState& s, double e) const {
            this->externalTransition(s, e);
            this->internalTransition(s);
        }

        // time_advance function
        [[nodiscard]] double timeAdvance(const dllState& state) const override {
            return state.sigma;
        }
    };

}

#endif