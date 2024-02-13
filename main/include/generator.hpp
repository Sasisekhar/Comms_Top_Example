#ifndef CADMIUM_EXAMPLE_GPT_GENERATOR_HPP_
#define CADMIUM_EXAMPLE_GPT_GENERATOR_HPP_

#include <include/cadmium/modeling/devs/atomic.hpp>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

namespace cadmium::comms::example {
	//! Class for representing the Generator DEVS model state.
	struct GeneratorState {
		uint64_t data;  //!< Current simulation time.
		double sigma;  //!< Time to wait before triggering the next internal transition function.

		//! Constructor function. It sets all the attributes to 0.
		GeneratorState(): data(0x346300002000), sigma(1.0) {}
	};

	/**
	 * Insertion operator for GeneratorState objects. It only displays the value of jobCount.
	 * @param out output stream.
	 * @param s state to be represented in the output stream.
	 * @return output stream with jobCount already inserted.
	 */
	std::ostream& operator<<(std::ostream& out, const GeneratorState& s) {
		out << s.data;
		return out;
	}

	//! Atomic DEVS model of a Job generator.
	class Generator : public Atomic<GeneratorState> {
        private:
            uint64_t dataPoints[3];
	    public:
		    Port<uint64_t> out;  //!< Output Port for sending new Job objects to be processed.

            /**
            * Constructor function for Generator DEVS model.
            * @param id model ID.
            * @param jobPeriod Job generation period.
            */
            Generator(const std::string& id): Atomic<GeneratorState>(id, GeneratorState())  {
                out = addOutPort<uint64_t>("out");
                dataPoints[0] = 0x346300000020;
                dataPoints[1] = 0x3c6300001020;
                dataPoints[2] = 0x346300002000;
                std::srand( (unsigned)time(NULL) );
                std::rand();
            }

            /**
            * Updates GeneratorState::clock and GeneratorState::sigma and increments GeneratorState::jobCount by one.
            * @param s reference to the current generator model state.
            */
            void internalTransition(GeneratorState& s) const override {
                s.data = dataPoints[std::rand() / ((RAND_MAX + 1u) / 3)];
                s.sigma = (double) std::rand() / ((RAND_MAX + 1u) / 2);
            }

            /**
            * Updates GeneratorState::clock and GeneratorState::sigma.
            * If it receives a true message via the Generator::inStop port, it passivates and stops generating Job objects.
            * @param s reference to the current generator model state.
            * @param e time elapsed since the last state transition function was triggered.
            * @param x reference to the atomic model input port set.
            */
            void externalTransition(GeneratorState& s, double e) const override {
                //nothing should happen here
            }

            /**
            * Sends a new Job that needs to be processed via the Generator::outGenerated port.
            * @param s reference to the current generator model state.
            * @param y reference to the atomic model output port set.
            */
            void output(const GeneratorState& s) const override {
                out->addMessage(s.data);
            }

            /**
            * It returns the value of GeneratorState::sigma.
            * @param s reference to the current generator model state.
            * @return the sigma value.
            */
            [[nodiscard]] double timeAdvance(const GeneratorState& s) const override {
                return s.sigma;
            }
	};
}  //namespace cadmium::example::gpt

#endif //CADMIUM_EXAMPLE_GPT_GENERATOR_HPP_