#include "include/cadmium/simulation/logger/stdout.hpp"
#include <include/cadmium/simulation/rt_root_coordinator.hpp>
#include <include/cadmium/simulation/rt_clock/chrono.hpp>
#include <limits>
#include "include/top.hpp"

using namespace cadmium::comms;

extern "C" {
	#ifdef RT_ESP32
		void app_main() //starting point for ESP32 code
	#else
		int main()		//starting point for simulation code
	#endif
	{
		std::shared_ptr<topSystem> model = std::make_shared<topSystem> ("topSystem");

		cadmium::ChronoClock clock;
		auto rootCoordinator = cadmium::RealTimeRootCoordinator<cadmium::ChronoClock<std::chrono::steady_clock>>(model, clock);

		rootCoordinator.setLogger<cadmium::STDOUTLogger>(";");

		rootCoordinator.start();
		rootCoordinator.simulate(std::numeric_limits<double>::infinity());
		// rootCoordinator.simulate(100.0);
		rootCoordinator.stop();	

		#ifndef RT_ESP32
						return 0;
		#endif
	}
}
