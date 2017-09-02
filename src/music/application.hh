#ifndef MUSIC_APPLICATION_HH
#include "music/music-config.hh"

#if MUSIC_USE_MPI
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <memory>
#include <mpi.h>
#include "music/configuration.hh"
#include "music/error.hh"
#include "music/port.hh"
#include "music/runtime.hh"
#include "music/connectivity.hh"
#include "music/port_manager.hh"
#include "music/music_luncher.hh"
#include "music/misc.hh"


namespace MUSIC
{

	enum class ApplicationState {RUNNING, STOPPED, FINALIZED};
	const double MUSIC_DEFAULT_TIMEBASE = 1e-9;

	class Application
	{
		private:
			Application(Configuration config, double timebase, MPI_Comm comm);

		public:
			Application ()
				: Application(0, nullptr, 1., MUSICLauncherFactory ().create ()) {}
			Application (int& argc, char**& argv,
					double timebase = MUSIC_DEFAULT_TIMEBASE,
					MUSICLauncher launcher = MUSICLauncherFactory ().create ());
			Application (int& argc, char**& argv, int required, int* provided,
					double timebase = MUSIC_DEFAULT_TIMEBASE,
					MUSICLauncher launcher = MUSICLauncherFactory ().create ());

			/* Application(Configuration config, double h, MPI::MPI_Comm comm); */

			Application(const Application&) = delete;
			Application& operator= (const Application&) = delete;

			// TODO move and move-assignment constructors

			void tick();
			void enterSimulationLoop();
			void exitSimulationLoop();
			void finalize();

			double time();
			double timebase() const;
			std::string applicationName() const;
			bool launchedByMusic () const;

			std::shared_ptr<ContInputPort> publishContInput (std::string identifier);
			std::shared_ptr<ContOutputPort> publishContOutput (std::string identifier);
			std::shared_ptr<EventInputPort> publishEventInput (std::string identifier);
			std::shared_ptr<EventOutputPort> publishEventOutput (std::string identifier);
			std::shared_ptr<MessageInputPort> publishMessageInput (std::string identifier);
			std::shared_ptr<MessageOutputPort> publishMessageOutput (std::string identifier);

		private:
			double timebase_;
			MPI::MPI_Comm comm_ {MPI::COMM_WORLD};
			int app_color_;
			int leader_;
			ApplicationState state_ {ApplicationState::STOPPED};
			bool launchedByMusic_ {false};
			std::unique_ptr<ApplicationMap> application_map_;
			PortConnectivityManager port_manager_;


		private:
			friend class Port;
			friend class temporal;

			Runtime runtime_;
			void assertValidState(std::string func_name, ApplicationState as);
			void initialize_MPI(int& argc, char**& argv, int required, int* provided);
			void initialize_MPI(int& argc, char**& argv);

			MPI::Intracomm communicator ();
			PortConnectivityManager& getPortConnectivityManager () const;
			int applicationColor();
			int nProcs () const;
			int leader () const;


	};

	inline double Application::time()
	{
		return runtime_.time();
	}

	inline void Application::tick()
	{
		assertValidState("tick", ApplicationState::RUNNING);
		runtime_.tick();
	}


	inline void Application::assertValidState(std::string func_name, ApplicationState as)
	{
		if (as == state_)
			return;
		std::stringstream ss;
		ss << "Calling " << func_name << " is permitted in "<< state_;
		std::string s = ss.str();
		errorRank(s);
	}

}
#endif
#define MUSIC_APPLICATION_HH
#endif
