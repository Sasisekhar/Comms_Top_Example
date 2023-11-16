#ifndef SASI_TOP_HPP
#define SASI_TOP_HPP

#include "cadmium/modeling/devs/coupled.hpp"
#include "ME.hpp"
#include "generator.hpp"
#include "tcl.hpp"
#include "dll.hpp"

namespace cadmium::comms {
    struct topSystem : public Coupled {

            /**
             * Constructor function for the blinkySystem model.
             * @param id ID of the blinkySystem model.
             */
            topSystem(const std::string& id) : Coupled(id) {
                auto generator = addComponent<Generator>("generator");
                auto layer1 = addComponent<tcl<uint64_t>>("layer1");
                auto layer2 = addComponent<dll>("layer2");
                auto phy = addComponent<ME>("phy", (gpio_num_t) 18, (gpio_num_t) 19, (uint32_t)80 * 1000 * 1000);

                addCoupling(generator->out, layer1->in);
                addCoupling(layer1->out, layer2->in);
                addCoupling(layer2->out, phy->in);
                // addCoupling(generator->out, phy->in);
            }
        };
}

#endif