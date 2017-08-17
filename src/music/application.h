#include <string>
#include <functional>
#include <iostream>
#include <sstream>

#include "music/music-config.hh"

#if MUSIC_USE_MPI
#include <mpi.h>
#include "music/configuration.hh"
#include "music/error.hh"


namespace MUSIC
{

	class enum ApplicationState {RUNNING, STOPPED, FINALIZED};


	class Application
	{
		public:
			Application(std::string name, double h);
			Application(std::string name, double h, MPI::MPI_Comm comm);
			// Apply Move symantics?
			Application(Configuration config, double h);
			Application(Configuration config, double h, MPI::MPI_Comm comm);

			double time() const;
			void tick();
			void enterSimulationLoop();
			void exitSimulationLoop();
			void finalize();


		private:
			/* void init(); */
			Configuration conf_;
			double h_;
			MPI::MPI_Comm comm_{MPI::COMM_WORLD};
			ApplicationState state_{ApplicationState::STOPPED};

		private:
			/* Runtime runtime_: */
			void assertValidState(std::string func_name, ApplicationState as);


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
