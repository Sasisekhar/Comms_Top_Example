#ifndef COMMS_TOP_HPP
#define COMMS_TOP_HPP

#include "cadmium/modeling/devs/coupled.hpp"
#include "ME.hpp"
#include "tcl.hpp"

namespace cadmium::comms {
    template<typename T>
    struct commstop : public Coupled {

        Port<T> in;
        Port<T> out;

        /**
         * Constructor function for the blinkySystem model.
         * @param id ID of the blinkySystem model.
         */
        commstop<T>(const std::string& id) : Coupled(id) {
            auto phy = addComponent<ME>("phy", (gpio_num_t) 18, (gpio_num_t) 19, (uint32_t)80 * 1000 * 1000);
            auto layer1 = addComponent<tcl<T>>("layer1");

            addCoupling(in, tcl->in);
            addCoupling(layer1->out, phy->in);
            addCoupling(phy->out, layer1->in);
            addCoupling(layer1->out, out);
        }
    };
}

#endif