#ifndef SASI_TOP_HPP
#define SASI_TOP_HPP

#include "cadmium/modeling/devs/coupled.hpp"
#include "commstop.hpp"
#include "RGB.hpp"
#include "led_output.hpp"

namespace cadmium::comms::example {
    struct topSystem : public Coupled {

            /**
             * Constructor function for the blinkySystem model.
             * @param id ID of the blinkySystem model.
             */
            topSystem(const std::string& id) : Coupled(id) {
                auto atomic_1 = addComponent<commstop<RGB>>("commstop_rx");
                auto atomic_2 = addComponent<led_output>("led_output");

                addCoupling(atomic_1->out, atomic_2->in);
            }
        };
}

#endif