#ifndef _GENERATOR_HPP__
#define _GENERATOR_HPP__

#include "cadmium/modeling/devs/atomic.hpp"

#ifndef NO_LOGGING
	#include <iostream>
#endif


#include <cstdlib>

namespace cadmium::comms {
	//! Class for representing the Generator DEVS model state.struct GeneratorState {
	struct GeneratorState {
		uint64_t val;
		double sigma;
		float deadline;
		//! Generator state constructor.
		GeneratorState(): val(0), sigma(1),  deadline(2.0)  {}
	};
#ifndef NO_LOGGING
		/**
		 * Insertion operator for GeneratorState objects. It only displays the value of sigma.
		 * @param out output stream.
		 * @param s state to be represented in the output stream.
		 * @return output stream with sigma already inserted.
		 */
		std::ostream& operator<<(std::ostream &out, const GeneratorState& state) {
			out << "Status: " << state.val; // state to string
			return out;
		}
#endif

	//! Atomic DEVS model of a Generator.
	class Generator : public Atomic<GeneratorState> {
	 private:
		
	 public:
		Port<uint64_t> out;

		/**
		 * Constructor function.
		 * @param id ID of the new Generator model object.
		 */
		Generator(const std::string& id): Atomic<GeneratorState>(id, GeneratorState()) {
			out = addOutPort<uint64_t>("out");
			state.val = 0;
			srand(0);
		}

		/**
		 * It updates the GeneratorState::sigma.
		 * @param state reference to the current state of the model.
		 */
		void internalTransition(GeneratorState& state) const override {
			// state.val = (uint32_t)rand();
			state.val = (uint64_t)9220358965123336698;
			// state.val = 0xAAAA1234;
		}

		/**
		 * Updates GeneratorState::state.
		 * @param state reference to the current model state.
		 * @param e time elapsed since the last state transition function was triggered.
		 * @param x reference to the model input port set.
		 */
		void externalTransition(GeneratorState& state, double e) const override {
			state.sigma = std::numeric_limits<double>::infinity();
		}

		/**
		 * @param state reference to the current model state.
		 * @param y reference to the atomic model output port set.
		 */
		void output(const GeneratorState& state) const override {
			out->addMessage(state.val);
//			printf("[generator] output function\n");
		}

		/**
		 * It returns the value of GeneratorState::sigma.
		 * @param state reference to the current model state.
		 * @return the sigma value.
		 */
		[[nodiscard]] double timeAdvance(const GeneratorState& state) const override {
			return state.sigma;
		}
	};
}  //namespace cadmium::blinkySystem

#endif //_GENERATOR_HPP__
