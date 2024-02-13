// #include <include/cadmium/simulation/rt_root_coordinator.hpp>
#include <include/cadmium/simulation/root_coordinator.hpp>
#include <limits>
#include "include/top.hpp"

#ifdef RT_ESP32
	#include <include/cadmium/simulation/rt_clock/ESPclock.hpp>
	#include "include/drivers/ESP_RMT_interrupt_handler.hpp"
#else
	// #include <include/cadmium/simulation/rt_clock/chrono.hpp>
#endif

#ifndef NO_LOGGING
	#include "include/cadmium/simulation/logger/stdout.hpp"
#endif

using namespace cadmium::comms::example;

// 346300000020
// 3c6300001020
// 346300002000
extern "C" {
	#ifdef RT_ESP32
		void app_main() //starting point for ESP32 code
	#else
		int main()		//starting point for simulation code
	#endif
	{

		auto model = std::make_shared<topSystem> ("top");

		#ifdef RT_ESP32
			cadmium::ESPclock<double, uint64_t, cadmium::comms::RMTinterruptHandler> clock(model);

			auto rootCoordinator = cadmium::RealTimeRootCoordinator<cadmium::ESPclock<double, uint64_t, cadmium::comms::RMTinterruptHandler>>(model, clock);
		#else
			// cadmium::ChronoClock clock;
			// auto rootCoordinator = cadmium::RealTimeRootCoordinator<cadmium::ChronoClock<std::chrono::steady_clock>>(model, clock);
			auto rootCoordinator = cadmium::RootCoordinator(model);
		#endif

		#ifndef NO_LOGGING
		// rootCoordinator.setLogger<cadmium::STDOUTLogger>(";");
		rootCoordinator.setLogger<cadmium::STDOUTLogger>(";");
		#endif

		rootCoordinator.start();
		// rootCoordinator.simulate(std::numeric_limits<double>::infinity());
		rootCoordinator.simulate(3.0);
		rootCoordinator.stop();	

		#ifndef RT_ESP32
						return 0;
		#endif
	}
}
