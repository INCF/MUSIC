#include <string>
#include <functional>
#include <iostream>
#include <sstream>

#include "music/music-config.hh"

#if MUSIC_USE_MPI
#include <mpi.h>
#include "music/configuration.hh"
#include "music/error.hh"
#include "music/port.hh"


namespace MUSIC
{

	class enum ApplicationState {RUNNING, STOPPED, FINALIZED};


	class Application
	{
		public:
			// Apply Move symantics?
			Application(Configuration config, double h);
			Application(Configuration config, double h, MPI::MPI_Comm comm);

			// TODO disable copy constructors etc

			double time() const;
			double timebase() const;
			void tick();
			void enterSimulationLoop();
			void exitSimulationLoop();
			void finalize();


			// Why pointer?
			ContInputPort* publishContInput (std::string identifier);
			ContOutputPort* publishContOutput (std::string identifier);
			EventInputPort* publishEventInput (std::string identifier);
			EventOutputPort* publishEventOutput (std::string identifier);
			MessageInputPort* publishMessageInput (std::string identifier);
			MessageOutputPort* publishMessageOutput (std::string identifier);

		private:
			/* void init(); */
			Configuration conf_;
			double h_;
			MPI::MPI_Comm comm_{MPI::COMM_WORLD};
			ApplicationState state_{ApplicationState::STOPPED};

			// Application should not manage this; this should be done by some
			// manager class
			std::vector<std::weak_ptr<Port>> ports_;

		private:
			friend class Port;

			Runtime* runtime_:
			void assertValidState(std::string func_name, ApplicationState as);
			void addPort(Port* p);
			MPI::Intracomm communicator ();


	};

	inline double Application::time() const
	{
		return runtime_->time();
	}

	inline void Application::tick()
	{
		assertValidState("tick", ApplicationState::RUNNING);
		runtime_->tick();
	}

	void assertValidState(std::string func_name, ApplicationState as)
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
