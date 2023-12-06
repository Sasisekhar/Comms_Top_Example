#ifndef TRAFFIC_HPP
#define TRAFFIC_HPP

#include <iostream>
#include "cadmium/modeling/devs/atomic.hpp"
#include "RGB.hpp"

namespace cadmium::comms::example {

    struct traffic_lightState {
        RGB signal;
        double sigma;
        double deadline;

        explicit traffic_lightState(): signal(255, 0, 0), sigma(1.0), deadline(1.0){
        }
    };

#ifndef NO_LOGGING
    std::ostream& operator<<(std::ostream &out, const traffic_lightState& state) {
        out << "Signal: " << state.signal;
        return out;
    }
#endif

    class traffic_light : public Atomic<traffic_lightState> {
        public:
        Port<RGB> out;

        traffic_light(const std::string id) : Atomic<traffic_lightState>(id, traffic_lightState()) {
            out = addOutPort<RGB>("out");
        }

        void internalTransition(traffic_lightState& state) const override {
            RGB R(32, 0, 0);
            RGB Y(32, 16, 0);
            RGB G(0, 32, 0);
            if(state.signal == R) {
                state.signal = Y;
            } else if(state.signal == Y) {
                state.signal = G;
            } else if(state.signal == G) {
                state.signal = R;
            } else {
                state.signal = R;
            }
        }

        // external transition
        void externalTransition(traffic_lightState& state, double e) const override {
            //Nothing should happen here
        }
        
        
        // output function
        void output(const traffic_lightState& state) const override {
            out->addMessage(state.signal);
        }

        // time_advance function
        [[nodiscard]] double timeAdvance(const traffic_lightState& state) const override {     
            //   return std::numeric_limits<double>::infinity();
            return state.sigma;
        }
    };

}

#endif