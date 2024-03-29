#ifndef COMMS_TOP_HPP
#define COMMS_TOP_HPP

#include "cadmium/modeling/devs/coupled.hpp"
#include "ME_rx.hpp"
#include "tcl.hpp"
#include "dll.hpp"

#ifndef RT_ESP32
using gpio_num_t = uint32_t;
#endif

namespace cadmium::comms {

    template<typename T>
    struct commstop : public Coupled {

        Port<uint64_t> in;
        Port<T> out;

        /**
         * Constructor function for the blinkySystem model.
         * @param id ID of the blinkySystem model.
         */
        commstop(const std::string& id) : Coupled(id) {
            in = this->template addInPort<uint64_t>("in");
            out = this->template addOutPort<T>("out");

            auto layer1 = addComponent<tcl<T>>("layer1");
            auto layer2 = addComponent<dll>("layer2");
            auto phy_rx = addComponent<ME_rx>("phy_rx", (gpio_num_t) 17, (uint32_t) 80 * 1000 * 1000);

            addCoupling(in, phy_rx->in);
            addCoupling(phy_rx->out, layer2->downstream_in);
            addCoupling(layer2->upstream_out, layer1->downstream_in);
            addCoupling(layer1->upstream_out, out);
        }
    };
}

#endif