/**
 * Real-time clock based on the chrono standard library.
 * Copyright (C) 2023  Román Cárdenas Rodríguez
 * ARSLab - Carleton University
 * GreenLSI - Polytechnic University of Madrid
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CADMIUM_SIMULATION_RT_CLOCK_ESP_HPP
#define CADMIUM_SIMULATION_RT_CLOCK_ESP_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "rt_clock.hpp"
#include "../../exception.hpp"
#include "interrupt_handler.hpp"
// #include "ESP_RMT_interrupt_handler.hpp"

// const char* TAG = "rt_clock.hpp";

namespace cadmium {
    /**
     * Real-time clock based on the std::chrono library. It is suitable for Linux, MacOS, and Windows.
     * @tparam T Internal clock type. By default, it uses the std::chrono::steady_clock
     */
    template<typename T = double, typename Y = uint64_t, typename Z = InterruptHandler<Y>>
    class ESPclock : RealTimeClock {
    private:
        gptimer_handle_t executionTimer;
        double rTimeLast;
        std::shared_ptr<Coupled> top_model;
        std::shared_ptr<InterruptHandler<Y>> ISR_handle;
     public:

        ESPclock() : RealTimeClock() {
            gptimer_config_t timer_config1 = {
                .clk_src = GPTIMER_CLK_SRC_DEFAULT,
                .direction = GPTIMER_COUNT_UP,
                .resolution_hz = 20 * 1000 * 1000, // 10MHz, 1 tick=100ns
            };
            ESP_ERROR_CHECK(gptimer_new_timer(&timer_config1, &executionTimer));

            ESP_ERROR_CHECK(gptimer_enable(executionTimer));
            gptimer_set_raw_count(executionTimer, 0);
            gptimer_start(executionTimer);
            this->top_model = NULL;
        }

        //! The empty constructor does not check the accumulated delay jitter.
        [[maybe_unused]] ESPclock(std::shared_ptr<Coupled> model) : RealTimeClock() {
            gptimer_config_t timer_config1 = {
                .clk_src = GPTIMER_CLK_SRC_DEFAULT,
                .direction = GPTIMER_COUNT_UP,
                .resolution_hz = 20 * 1000 * 1000, // 10MHz, 1 tick=100ns
            };
            ESP_ERROR_CHECK(gptimer_new_timer(&timer_config1, &executionTimer));

            ESP_ERROR_CHECK(gptimer_enable(executionTimer));
            gptimer_set_raw_count(executionTimer, 0);
            gptimer_start(executionTimer);
            this->top_model = model;
            this->ISR_handle = std::make_shared<Z>();
        }

        /**
         * Starts the real-time clock.
         * @param timeLast initial simulation time.
         */
        void start(double timeLast) override {
            RealTimeClock::start(timeLast);
            uint64_t count = 0;
            uint32_t res = 0;
            gptimer_get_resolution(executionTimer, &res);
            gptimer_get_raw_count(executionTimer, &count);
            rTimeLast = (double)count / (double)res;
        }

        /**
         * Stops the real-time clock.
         * @param timeLast last simulation time.
         */
        void stop(double timeLast) override {
            uint64_t count = 0;
            uint32_t res = 0;
            gptimer_get_resolution(executionTimer, &res);
            gptimer_get_raw_count(executionTimer, &count);
            rTimeLast = (double)count / (double)res;
            RealTimeClock::stop(timeLast);
        }

        /**
         * Waits until the next simulation time or until an external event happens.
         *
         * @param nextTime next simulation time (in seconds) for an internal transition.
         * @return next simulation time (in seconds). Return value must be less than or equal to nextTime.
         * */
        double waitUntil(double timeNext) override {
            auto duration = timeNext - vTimeLast;
            rTimeLast += duration;

            uint64_t count = 0;
            uint32_t res = 0;
            gptimer_get_resolution(executionTimer, &res);
            gptimer_get_raw_count(executionTimer, &count);
            double timeNow = (double)count / (double)res;

            cadmium::Component pseudo("pseudo");
            cadmium::Port<Y> out;
            out = pseudo.addOutPort<Y>("out");
            

            while(timeNow < rTimeLast) {
                gptimer_get_resolution(executionTimer, &res);
                gptimer_get_raw_count(executionTimer, &count);
                timeNow = (double)count / (double)res;

                if (ISR_handle->ISRcb()) {
                    auto data = ISR_handle->decodeISR();
                    out->addMessage(data);
                    top_model->getInPort("in")->propagate(out);
                    break;
                }
                
                
            }
            gptimer_get_resolution(executionTimer, &res);
            gptimer_get_raw_count(executionTimer, &count);
            timeNow = (double)count / (double)res;

            rTimeLast = timeNow;

#ifdef DEBUG_DELAY
            std::cout << "[DELAY] " << (timeNow - rTimeLast)*(1000 * 1000) << " us" << std::endl;
#endif
            return RealTimeClock::waitUntil(std::min(timeNow, timeNext));
        }
    };
}

#endif // CADMIUM_SIMULATION_RT_CLOCK_CHRONO_HPP
