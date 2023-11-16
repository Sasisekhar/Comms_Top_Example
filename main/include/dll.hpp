#ifndef DLL_HPP
#define DLL_HPP

#include "cadmium/modeling/devs/atomic.hpp"
#include "dataframe.hpp"
#include "esp_log.h"
#include <iostream>

namespace cadmium {

    struct dllState {
        dataframe inData;
        uint32_t outData;
        size_t frame_num;
        double sigma;
        double deadline;

        /**
         * Processor state constructor. By default, the processor is idling.
         * 
         */
        explicit dllState(): inData(), outData(0), frame_num(0), sigma(0.1), deadline(1.0){
        }
    };

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

    class dll : public Atomic<dllState> {
        public:
        Port<dataframe> in;
        Port<uint32_t> out;

        dll(const std::string id) : Atomic<dllState>(id, dllState()) {
            in = Component::addInPort<dataframe>("in");
            out = Component::addOutPort<uint32_t>("out");
        }

        void internalTransition(dllState& state) const override {
            if(state.frame_num < state.inData.data.size()) {
                ESP_LOGI("[DEBUG]", "Counter = %d", state.frame_num);
                state.outData = state.inData.data[state.frame_num++];
                // ESP_LOGI("DLL", "%lx", state.outData);
            } else {
                state.sigma = std::numeric_limits<double>::infinity();
            }
        }

        // external transition
        void externalTransition(dllState& state, double e) const override {
            if(!in->empty()){
                for( const auto& x : in->getBag()){
                    state.inData = x;
                }
            }
            state.frame_num = 0;
            state.sigma = 0.3;
        }
        
        
        // output function
        void output(const dllState& state) const override {
            out->addMessage(state.outData);
        }

        // time_advance function
        [[nodiscard]] double timeAdvance(const dllState& state) const override {
            return state.sigma;
        }
    };

}

#endif