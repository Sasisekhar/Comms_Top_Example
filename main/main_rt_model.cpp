#include <include/cadmium/simulation/rt_root_coordinator.hpp>
#include <limits>
#include "include/top.hpp"

#ifdef RT_ESP32
	#include <include/cadmium/simulation/rt_clock/ESPclock.hpp>
#else
	#include <include/cadmium/simulation/rt_clock/chrono.hpp>
#endif

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

		std::shared_ptr<topSystem> model = std::make_shared<topSystem> ("topSystem");

		#ifdef RT_ESP32
			cadmium::ESPclock clock;
			auto rootCoordinator = cadmium::RealTimeRootCoordinator<cadmium::ESPclock<double>>(model, clock);
		#else
			cadmium::ChronoClock clock;
			auto rootCoordinator = cadmium::RealTimeRootCoordinator<cadmium::ChronoClock<std::chrono::steady_clock>>(model, clock);
		#endif

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
