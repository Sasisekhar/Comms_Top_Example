#ifndef DLL_HPP
#define DLL_HPP

#include "cadmium/modeling/devs/atomic.hpp"
#include "tcl_packet.hpp"
#include "dll_frame.hpp"
#include "comms_defines.hpp"
#include <iostream>

namespace cadmium::comms {

    struct dllState {
        tcl_packet upstream_in_data;
        dll_frame downstream_out_data;
        dll_frame downstream_in_data;
        tcl_packet upstream_out_data;
        bool downstream_tx;
        bool upstream_tx;
        size_t frame_num;
        std::vector<dll_frame> downstream_rx_hist;
        std::vector<uint32_t> data;
        double sigma;
        double deadline;

        /**
         * Processor state constructor. By default, the processor is idling.
         * 
         */
        explicit dllState(): upstream_in_data(), downstream_out_data(), downstream_in_data(), upstream_out_data(), downstream_tx(false), upstream_tx(false), frame_num(0), downstream_rx_hist(), data(0), sigma(std::numeric_limits<double>::infinity()), deadline(1.0){
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
        out << "{ upstream_in_data: " << std::hex << state.upstream_in_data  << "}";
        return out;
    }
#endif

    class dll : public Atomic<dllState> {
        private:

        static uint8_t computeChecksum(dll_frame frame) {
            uint8_t checksum = 0;
            for(int i = 0; i < PAYLOAD_LEN; i++) {
                checksum += ((frame.data & ((0x00000001) << i)) >> i);
            }

            for(int i = 0; i < FRAME_NUM_LEN; i++) {
                checksum += ((frame.frame_num & ((0x01) << i)) >> i);
            }

            for(int i = 0; i < FRAME_NUM_LEN; i++) {
                checksum += ((frame.total_frames & ((0x01) << i)) >> i);
            }

            for(int i = 0; i < SELECT_LEN; i++) {
                checksum += (((frame.datalen_frame_select? 1 : 0 )& ((0x01) << i)) >> i);
            }
            return checksum;
        }

        public:
        Port<tcl_packet> upstream_in;
        Port<dll_frame> downstream_out;

        Port<dll_frame> downstream_in;
        Port<tcl_packet> upstream_out;

        dll(const std::string id) : Atomic<dllState>(id, dllState()) {
            upstream_in = addInPort<tcl_packet>("upstream_in");
            downstream_out = addOutPort<dll_frame>("downstream_out");

            downstream_in = addInPort<dll_frame>("downstream_in");
            upstream_out = addOutPort<tcl_packet>("upstream_out");

            state.downstream_rx_hist.clear();
        }

        void internalTransition(dllState& state) const override {
            state.upstream_tx = false;
            if(state.frame_num < state.data.size()) {
                state.downstream_out_data.datalen_frame_select = false;
                state.downstream_out_data.data = state.data[state.frame_num];
                state.downstream_out_data.frame_num = state.frame_num;
                state.downstream_out_data.checksum = computeChecksum(state.downstream_out_data);

                // std::cout << "\x1B[33mdelInt->framenum: " << state.frame_num << "\033[0m" << std::endl;

                state.frame_num++;
                state.downstream_tx = true;
            } else {
                state.sigma = std::numeric_limits<double>::infinity();
                state.downstream_tx = false;
            }
        }

        // external transition
        void externalTransition(dllState& state, double e) const override {
            if(!upstream_in->empty()){
                state.upstream_in_data.data.clear();

                for( const auto& x : upstream_in->getBag()){
                    state.upstream_in_data = x;
                }
                state.frame_num = 0;

                if(state.upstream_in_data.data.size() < sizeof(uint32_t)) {

                    state.downstream_out_data.datalen_frame_select = true;
                    
                    state.downstream_out_data.data = 0;
                    for(int i = 0; i < state.upstream_in_data.data.size(); i++) {
                        state.downstream_out_data.data |= state.upstream_in_data.data[i] << i * 8;
                    }
                    state.downstream_out_data.frame_num = state.upstream_in_data.data.size();
                    state.downstream_out_data.total_frames = state.upstream_in_data.data.size();
                    state.downstream_out_data.checksum = computeChecksum(state.downstream_out_data);

                    state.frame_num = state.upstream_in_data.data.size();
                    state.downstream_tx = true;
                } else {
                    uint32_t tmp_data = 0;
                    state.data.clear();
                    if((state.upstream_in_data.data.size() % 4) == 0){
                        for(int i = 0; i < state.upstream_in_data.data.size(); i++) {
                            tmp_data |= state.upstream_in_data.data[i] << (i % 4) * 8;
                            if((i % 4 == 3)) {
                                state.data.push_back(tmp_data);
                                tmp_data = 0UL;
                            }
                        }
                    } else {
                        for(int i = 0; i < (state.upstream_in_data.data.size() + 4 + (state.upstream_in_data.data.size() % 4)); i++) {
                            if(i < state.upstream_in_data.data.size()) {
                                tmp_data |= state.upstream_in_data.data[i] << (i % 4) * 8;
                            } else {
                                tmp_data |= 0 << (i % 4) * 8;
                            }
                            
                            if((i % 4 == 3)) {
                                state.data.push_back(tmp_data);
                                tmp_data = 0UL;
                            }
                        }
                    }
                }

                if(state.frame_num < state.data.size()) {
                    state.downstream_out_data.datalen_frame_select = false;
                    state.downstream_out_data.data = state.data[state.frame_num];
                    state.downstream_out_data.frame_num = state.frame_num;
                    state.downstream_out_data.total_frames = state.data.size();
                    state.downstream_out_data.checksum = computeChecksum(state.downstream_out_data);
                    state.frame_num++;
                    state.downstream_tx = true;
                }

                state.sigma = 0.02;
            }

            if(!downstream_in->empty()) {

                for( const auto& x : downstream_in->getBag()){
                    state.downstream_in_data = x;
                }
                
                bool checksum_valid = true;
                if(state.downstream_in_data.checksum == computeChecksum(state.downstream_in_data)){
                    state.downstream_rx_hist.push_back(state.downstream_in_data);
                    checksum_valid = true;
                } else {
                    std::cout << "CHECKSUM INVALID" << std::endl;
                }

                if(checksum_valid){
                    state.upstream_out_data.data.clear();
                    if(state.downstream_in_data.datalen_frame_select) {
                        for(int i = 0; i <  state.downstream_out_data.frame_num; i++){
                            state.upstream_out_data.data.push_back((state.downstream_in_data.data & (0xFFULL << i*8)) >> i*8);
                        }
                        state.upstream_tx = true; //move this to the bottom after fixing checksum check
                        state.sigma = 0;//move this to the bottom after fixing checksum check
                    } else if(state.downstream_rx_hist.back().frame_num == (state.downstream_rx_hist.back().total_frames - 1)) {
                        
                        for(auto x : state.downstream_rx_hist) {
                            for(int i = 0; i <  4; i++){
                                state.upstream_out_data.data.push_back((x.data & (0xFFULL << i*8)) >> i*8);
                            }
                        }
                        state.downstream_rx_hist.clear();
                        state.upstream_tx = true;//move this to the bottom after fixing checksum check
                        state.sigma = 0;//move this to the bottom after fixing checksum check
                    }
                }
            }
        }
        
        
        // output function
        void output(const dllState& state) const override {
            if(state.downstream_tx) {
                downstream_out->addMessage(state.downstream_out_data);
            }

            if(state.upstream_tx){
                upstream_out->addMessage(state.upstream_out_data);
            }
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