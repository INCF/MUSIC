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
#include "music/music_context.hh"
#include "music/port_manager.hh"
#include "music/misc.hh"


namespace MUSIC
{

	enum class ApplicationState {RUNNING, STOPPED, FINALIZED};
	const double MUSIC_DEFAULT_TIMEBASE = 1e-9;
	class PortConnectivityManager;

	class Application final
	{
		private:
			Application(std::unique_ptr<Configuration> config, MPI::Intracomm comm, bool launchedByMusic, double timebase);

		public:
			Application ();
			Application (std::unique_ptr<MusicContext> context,
					double timebase = MUSIC_DEFAULT_TIMEBASE);
			Application (int& argc, char**& argv, double timebase = MUSIC_DEFAULT_TIMEBASE);
			Application (int& argc, char**& argv, int required, int* provided, double timebase = MUSIC_DEFAULT_TIMEBASE);

			Application(const Application&) = delete;
			Application& operator= (const Application&) = delete;
			~Application ();

			// TODO move and move-assignment constructors

			void tick();
			void enterSimulationLoop (double h);
			void exitSimulationLoop ();
			void finalize();


			template <class PortT>
			std::shared_ptr<PortT> publish (std::string identifier)
			{
				return port_manager_.createPort<PortT> (*this, identifier);
			}

			PortConnectivityManager& getPortConnectivityManager ();

		    bool config (std::string var, double* result);
			bool config (std::string var, int* result);
		    bool config (std::string var, std::string* result);

			double time() const;
			double timebase() const;
			std::string name () const;
			bool launchedByMusic () const;
			MPI::Intracomm communicator () const;

			/* std::shared_ptr<ContInputPort> publishContInput (std::string identifier); */
			/* std::shared_ptr<ContOutputPort> publishContOutput (std::string identifier); */
			/* std::shared_ptr<EventInputPort> publishEventInput (std::string identifier); */
			/* std::shared_ptr<EventOutputPort> publishEventOutput (std::string identifier); */
			/* std::shared_ptr<MessageInputPort> publishMessageInput (std::string identifier); */
			/* std::shared_ptr<MessageOutputPort> publishMessageOutput (std::string identifier); */

		private:
			std::unique_ptr<Configuration> config_;
			double timebase_;
			MPI::Intracomm comm_ {MPI::COMM_WORLD};
			ApplicationState state_ {ApplicationState::STOPPED};
			bool launchedByMusic_ {false};
			/* std::unique_ptr<ApplicationMap> application_map_; */
			PortConnectivityManager port_manager_;


		private:
			friend class Port;
			friend class TemporalNegotiator;
			friend class Runtime;

			std::unique_ptr<Runtime> runtime_;
			Clock localTime_;
			void assertValidState(std::string func_name, ApplicationState as);

			const ApplicationMap& applicationMap () const;
			int color () const;
			int nProcs () const;
			int leader () const;


	};


	inline void Application::tick()
	{
		assertValidState("tick", ApplicationState::RUNNING);
		runtime_->tick();
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
