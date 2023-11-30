#ifndef TCL_HPP
#define TCL_HPP

#include <iostream>
#include "cadmium/modeling/devs/atomic.hpp"
#include "tcl_packet.hpp"
// #include "esp_log.h"

namespace cadmium::comms {

    template<typename T>
    struct tclState {
        T upstream_in_data;
        tcl_packet downstream_out_data;
        tcl_packet downstream_in_data;
        T upstream_out_data;
        bool upstream_tx;
        bool downstream_tx;
        double sigma;
        double deadline;

        /**
         * Processor state constructor. By default, the processor is idling.
         * 
         */
        explicit tclState<T>(): upstream_in_data(), downstream_out_data(), downstream_in_data(), upstream_out_data(), upstream_tx(false), downstream_tx(false), sigma(std::numeric_limits<double>::infinity()), deadline(1.0){
        }
    };

#ifndef NO_LOGGING
    /**
     * Insertion operator for GeneratorState objects. It only displays the value of sigma.
     * @param out output stream.
     * @param s state to be represented in the output stream.
     * @return output stream with sigma already inserted.
     */
    template<typename T>
    std::ostream& operator<<(std::ostream &out, const tclState<T>& state) {
        out << "upstream_in_data: " << std::hex << state.upstream_in_data << ", downstream_out_data: { ";
        for(auto x : state.downstream_out_data.data){
            out << std::hex << x;
            out << ", ";
        }
         out << " }";
        return out;
    }
#endif

    template<typename T>
    class tcl : public Atomic<tclState<T>> {
        public:
        Port<T> upstream_in;
        Port<tcl_packet> downstream_out;

        Port<tcl_packet> downstream_in;
        Port<T> upstream_out;

        tcl(const std::string id) : Atomic<tclState<T>>(id, tclState<T>()) {
            upstream_in = this->template addInPort<T>("upstream_in");
            downstream_out = this->template addOutPort<tcl_packet>("downstream_out");

            downstream_in = this->template addInPort<tcl_packet>("downstream_in");
            upstream_out = this->template addOutPort<T>("upstream_out");
        }

        void internalTransition(tclState<T>& state) const override {
            state.downstream_tx = false;
            state.upstream_tx = false;
            state.sigma = std::numeric_limits<double>::infinity();
        }

        // external transition
        void externalTransition(tclState<T>& state, double e) const override {
            if(!upstream_in->empty()){
                for( const auto x : upstream_in->getBag()){
                    state.upstream_in_data = x;
                }

                state.downstream_out_data.data.clear();

                const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&state.upstream_in_data);
                size_t data_len = sizeof(T) / sizeof(uint8_t);
                // std::cout << (unsigned int)sizeof(T) << std::endl;

                for (size_t i = 0; i < data_len; i++) {
                    state.downstream_out_data.data.push_back(*ptr);
                    ++ptr;
                }

                state.downstream_tx = true;
                state.sigma = 0;
            }

            if(!downstream_in->empty()) {
                for( const auto &x : downstream_in->getBag()){
                    state.downstream_in_data = x;
                }

                uint8_t* ptr = reinterpret_cast<uint8_t*>(&state.upstream_out_data);
                size_t data_len = sizeof(T) / sizeof(uint8_t);

                if (state.downstream_in_data.data.size() >= data_len) {
                    for (size_t i = 0; i < data_len; ++i) {
                        *ptr = state.downstream_in_data.data[i];
                        ++ptr;
                    }

                } else {
                    // Handle the case where the input vector doesn't have enough elements.
                    // maybe throw an error? fill default values?...idk
                    std::cout << "Size doesn't match" << std::endl;
                }

                state.upstream_tx = true;
                state.sigma = 0;

            }
        }
        
        
        // output function
        void output(const tclState<T>& state) const override {
            if(state.downstream_tx){
                downstream_out->addMessage(state.downstream_out_data);
            }

            if(state.upstream_tx){
                upstream_out->addMessage(state.upstream_out_data);
            }
        }

        // time_advance function
        [[nodiscard]] double timeAdvance(const tclState<T>& state) const override {     
            //   return std::numeric_limits<double>::infinity();
            return state.sigma;
        }
    };

}

#endif