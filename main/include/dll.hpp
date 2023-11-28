#ifndef DLL_HPP
#define DLL_HPP

#include "cadmium/modeling/devs/atomic.hpp"
#include "dataframe.hpp"
#include "esp_log.h"
#include <iostream>

namespace cadmium {

    struct dllState {
        dataframe inData;
        uint64_t outData;
        size_t frame_num;
        double sigma;
        double deadline;

        /**
         * Processor state constructor. By default, the processor is idling.
         * 
         */
        explicit dllState(): inData(), outData(0), frame_num(0), sigma(std::numeric_limits<double>::infinity()), deadline(1.0){
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
        out << "inData: " << std::hex << state.inData;
        return out;
    }
#endif

    class dll : public Atomic<dllState> {
        private:

        public:
        Port<dataframe> in;
        Port<uint64_t> out;

        dll(const std::string id) : Atomic<dllState>(id, dllState()) {
            in = Component::addInPort<dataframe>("in");
            out = Component::addOutPort<uint64_t>("out");
        }

        void internalTransition(dllState& state) const override {
            if(state.frame_num < state.inData.data.size()) {
                uint64_t out = 0;
                uint8_t checksum = 0;
                for(int i = 0; i < 32; i++) { //!ATTENTION hardcoded value of 32
                    checksum += ((state.inData.data[state.frame_num] & ((0x00000001) << i)) >> i);
                }

                for(int i = 37; i >= 0; i--) {
                    uint64_t tmp = 0;
                    if(i < 32) {
                        tmp = ((state.inData.data[state.frame_num] & ((0x00000001) << i)) >> i);
                    } else {
                        tmp = ((checksum & ((0x01) << (i - 32))) >> (i - 32));
                    }

                    out |= tmp << i;
                }
                state.outData = out;
                state.frame_num++;
            } else {
                state.sigma = std::numeric_limits<double>::infinity();
            }
        }

        // external transition
        void externalTransition(dllState& state, double e) const override {
            state.inData.data.clear();
            if(!in->empty()){
                for( const auto& x : in->getBag()){
                    state.inData = x;
                }
            }
            state.frame_num = 0;
            state.sigma = 0.02;

            if(state.frame_num < state.inData.data.size()) {
                uint64_t out = 0;
                uint8_t checksum = 0;
                for(int i = 0; i < 32; i++) { //!ATTENTION hardcoded value of 32
                    checksum += ((state.inData.data[state.frame_num] & ((0x00000001) << i)) >> i);
                }

                for(int i = 37; i >= 0; i--) {
                    uint64_t tmp = 0;
                    if(i < 32) {
                        tmp = ((state.inData.data[state.frame_num] & ((0x00000001) << i)) >> i);
                    } else {
                        tmp = ((checksum & ((0x01) << (i - 32))) >> (i - 32));
                    }

                    out |= tmp << i;
                }
                state.outData = out;
                state.frame_num++;
            } else {
                state.sigma = std::numeric_limits<double>::infinity();
            }
        }
        
        
        // output function
        void output(const dllState& state) const override {
            // ESP_LOGI("[DLL LAMBDA]", "Sent: 0x%llx", state.outData);
            out->addMessage(state.outData);
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