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

	ContInputPort*
	Application::publishContInput (std::string identifier)
	{
	return new ContInputPort (this, identifier);
	}


	ContOutputPort*
	Application::publishContOutput (std::string identifier)
	{
	return new ContOutputPort (this, identifier);
	}


	EventInputPort*
	Application::publishEventInput (std::string identifier)
	{
	return new EventInputPort (this, identifier);
	}


	EventOutputPort*
	Application::publishEventOutput (std::string identifier)
	{
	return new EventOutputPort (this, identifier);
	}


	MessageInputPort*
	Application::publishMessageInput (std::string identifier)
	{
	return new MessageInputPort (this, identifier);
	}


	MessageOutputPort*
	Application::publishMessageOutput (std::string identifier)
	{
	return new MessageOutputPort (this, identifier);
	}




}

#endif

