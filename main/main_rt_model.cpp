#include <include/cadmium/simulation/rt_root_coordinator.hpp>
#include <include/cadmium/simulation/rt_clock/chrono.hpp>
#include <limits>
#include "include/top.hpp"


#ifndef NO_LOGGING
	#include "include/cadmium/simulation/logger/stdout.hpp"
#endif

using namespace cadmium::comms::example;

extern "C" {
	#ifdef RT_ESP32
		void app_main() //starting point for ESP32 code
	#else
		int main()		//starting point for simulation code
	#endif
	{
		// configure_led();
		// blink_led();
        // /* Toggle the LED state */
        // s_led_state = !s_led_state;
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
		// blink_led();
        // /* Toggle the LED state */
        // s_led_state = !s_led_state;
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
		// blink_led();
        // /* Toggle the LED state */
        // s_led_state = !s_led_state;
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

		std::shared_ptr<topSystem> model = std::make_shared<topSystem> ("topSystem");

		cadmium::ChronoClock clock/*(std::chrono::milliseconds(900))*/;
		auto rootCoordinator = cadmium::RealTimeRootCoordinator<cadmium::ChronoClock<std::chrono::steady_clock>>(model, clock);

		#ifndef NO_LOGGING
		rootCoordinator.setLogger<cadmium::STDOUTLogger>(";");
		#endif

		rootCoordinator.start();
		rootCoordinator.simulate(std::numeric_limits<double>::infinity());
		// rootCoordinator.simulate(5.99);
		rootCoordinator.stop();	

		#ifndef RT_ESP32
						return 0;
		#endif
	}
}
