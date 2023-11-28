#ifndef RT_DIGITALINPUT_TEST_HPP
#define RT_DIGITALINPUT_TEST_HPP

#include <iostream>
#include <driver/gpio.h>
#include "cadmium/modeling/devs/atomic.hpp"

namespace cadmium::comms {
  
  struct DigitalInputState {
      bool level;
      uint64_t output;
      bool last;
      double sigma;

      /**
      * Processor state constructor. By default, the processor is idling.
      * 
      */
      explicit DigitalInputState(): level(false), output(0), last(false), sigma(0){
      }

  }; 

  /**
     * Insertion operator for ProcessorState objects. It only displays the value of sigma.
     * @param out output stream.
     * @param s state to be represented in the output stream.
     * @return output stream with sigma already inserted.
     */
    
    std::ostream& operator<<(std::ostream &out, const DigitalInputState& state) {
        out << "Pin: " << (state.level ? 1 : 0); 
        return out;
    }

  class DigitalInput : public Atomic<DigitalInputState> {
      public:
      
        Port<uint64_t> out;
        gpio_num_t inPin;
        double   pollingRate;
        // default constructor
        DigitalInput(const std::string& id, gpio_num_t pin): Atomic<DigitalInputState>(id, DigitalInputState())  {
          out = addOutPort<uint64_t>("out");
          inPin = pin;
          gpio_reset_pin(inPin);
          gpio_set_direction(inPin, GPIO_MODE_INPUT);
          pollingRate = 1; 
          state.output = (uint64_t) 9220358965123336698;
          state.last = state.level;
        };
      
      // internal transition
      void internalTransition(DigitalInputState& state) const override {
        state.last = state.level;
        state.level = (gpio_get_level(inPin) == 1)? true : false;
        state.sigma = pollingRate;
      }

      // external transition
      void externalTransition(DigitalInputState& state, double e) const override {
        // MBED_ASSERT(false);
        // throw std::logic_error("External transition called in a model with no input ports");
      }
      
      // output function
      void output(const DigitalInputState& state) const override {
        if(state.last != state.level) {
          out->addMessage(state.output);
        }
      }

      // time_advance function
      [[nodiscard]] double timeAdvance(const DigitalInputState& state) const override {     
          return state.sigma;
      }

  };
} 

#endif // BOOST_SIMULATION_PDEVS_DIGITALINPUT_HPP
