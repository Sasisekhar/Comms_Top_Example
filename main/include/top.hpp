#ifndef SASI_TOP_HPP
#define SASI_TOP_HPP

#include "cadmium/modeling/devs/coupled.hpp"
#include "generator.hpp"
#include "commstop.hpp"
#include "trial_port_type.hpp"

namespace cadmium::example {
    struct topSystem : public Coupled {

            /**
             * Constructor function for the blinkySystem model.
             * @param id ID of the blinkySystem model.
             */
            topSystem(const std::string& id) : Coupled(id) {
                // auto generator = addComponent<Generator>("generator");
                // auto comms = addComponent<cadmium::comms::commstop<bool>>("commstop");
                auto comms = addComponent<cadmium::comms::commstop<RGB_val>>("commstop");

                // addCoupling(generator->out, comms->in);
            }
        };
}

#endif