#ifndef TCL_HPP
#define TCL_HPP

#include <iostream>
#include "cadmium/modeling/devs/atomic.hpp"
#include "dataframe.hpp"
#include "esp_log.h"

namespace cadmium::comms {

    template<typename T>
    struct tclState {
        T inData;
        dataframe outData;
        double sigma;
        double deadline;

        /**
         * Processor state constructor. By default, the processor is idling.
         * 
         */
        explicit tclState<T>(): inData(), outData(), sigma(0.1), deadline(1.0){
        }
    };

    /**
     * Insertion operator for GeneratorState objects. It only displays the value of sigma.
     * @param out output stream.
     * @param s state to be represented in the output stream.
     * @return output stream with sigma already inserted.
     */
    template<typename T>
    std::ostream& operator<<(std::ostream &out, const tclState<T>& state) {
        out << "inData: " << std::hex << state.inData << ", outData: { ";
        for(auto x : state.outData.data){
            out << std::hex << x;
            out << ", ";
        }
         out << " }";
        return out;
    }

    template<typename T>
    class tcl : public Atomic<tclState<T>> {
        public:
        Port<T> in;
        Port<dataframe> out;

        tcl(const std::string id) : Atomic<tclState<T>>(id, tclState<T>()) {
            in  = this->template addInPort<T>("in");
            out = this->template addOutPort<dataframe>("out");
        }

        void internalTransition(tclState<T>& state) const override {

            state.outData.data.clear();
            for(size_t i = 0; i < sizeof(T)/sizeof(uint32_t); i++){
                uint32_t tmp1;
                uint32_t tmp2 = state.inData >> 32 * i;
                memcpy(&tmp1, &tmp2, sizeof(uint32_t));
                state.outData.data.push_back(tmp1);
                // state.outData.data[i] = tmp1;
            }
            state.sigma = std::numeric_limits<double>::infinity();
        }

        // external transition
        void externalTransition(tclState<T>& state, double e) const override {
            if(!in->empty()){
                for( const auto x : in->getBag()){
                    state.inData = x;
                }
            }
            state.sigma = 0.1;
        }
        
        
        // output function
        void output(const tclState<T>& state) const override {
            out->addMessage(state.outData);
        }

        // time_advance function
        [[nodiscard]] double timeAdvance(const tclState<T>& state) const override {     
            //   return std::numeric_limits<double>::infinity();
            return state.sigma;
        }
    };

}

#endif