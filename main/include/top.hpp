#ifndef SASI_TOP_HPP
#define SASI_TOP_HPP

#include "cadmium/modeling/devs/coupled.hpp"
#include "generator.hpp"
#include "commstop.hpp"

namespace cadmium::comms {
    struct topSystem : public Coupled {

            /**
             * Constructor function for the blinkySystem model.
             * @param id ID of the blinkySystem model.
             */
            topSystem(const std::string& id) : Coupled(id) {
                auto generator = addComponent<Generator>("generator");
                auto comms = addComponent<commstop<uint64_t>>("commstop");

                // addCoupling(din->out, comms->in);
                addCoupling(generator->out, comms->in);
            }
        };
}

#endif