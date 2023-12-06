#ifndef _LED_OUTPUT_HPP
#define _LED_OUTPUT_HPP

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include <iostream>
#include "cadmium/modeling/devs/atomic.hpp"

namespace cadmium::comms::example {

    static led_strip_handle_t led_strip;

    struct led_outputState {
        RGB colour;
        double sigma;
        double deadline;

        explicit led_outputState(): colour(255, 0, 0), sigma(std::numeric_limits<double>::infinity()), deadline(1.0){
        }
    };

#ifndef NO_LOGGING
    std::ostream& operator<<(std::ostream &out, const led_outputState& state) {
        out << "colour: " << state.colour;
        return out;
    }
#endif

class led_output : public Atomic<led_outputState> {
        public:
        Port<RGB> in;

        led_output(const std::string id) : Atomic<led_outputState>(id, led_outputState()) {
            in = addInPort<RGB>("in");

            led_strip_config_t strip_config = {
                .strip_gpio_num = 48,
                .max_leds = 1, // at least one LED on board
            };

            led_strip_rmt_config_t rmt_config = {
                .resolution_hz = 10 * 1000 * 1000, // 10MHz
            };
            rmt_config.flags.with_dma = false;
            ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

            led_strip_clear(led_strip);
        }

        void internalTransition(led_outputState& state) const override {
            state.sigma = std::numeric_limits<double>::infinity();
        }

        // external transition
        void externalTransition(led_outputState& state, double e) const override {
           if(!in->empty()) {
            for(const auto &x : in->getBag()) {
                state.colour = x;
            }
            state.sigma = 0;
           }
        }
        
        
        // output function
        void output(const led_outputState& state) const override {
            led_strip_set_pixel(led_strip, 0, state.colour.R, state.colour.G, state.colour.B);
            led_strip_refresh(led_strip);
        }

        // time_advance function
        [[nodiscard]] double timeAdvance(const led_outputState& state) const override {     
            //   return std::numeric_limits<double>::infinity();
            return state.sigma;
        }
    };
}
#endif