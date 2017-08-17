#include "music/application.hh"
#if MUSIC_USE_MPI
#include <mpi.h>

namespace MUSIC
{
    static std::string err_not_runtime = "Application not in running state";

	Application::Application(Configuration config, double h):
		conf_(config), h_(h)
	{
	}

	Application::Application(Configuration config, double h, MPI::MPI_Comm comm):
		Application(config, h), comm_(comm)
	{
	}

	void Application::enterSimulationLoop()
	{
		state_ = SimulationState::RUNNING;
		// Eventually run negotiators etc. again
	}

	void Application::exitSimulationLoop()
	{
		state_ = SimulationState::STOPPED;
	}

	void Application::finalize()
	{
	}





}

#endif

