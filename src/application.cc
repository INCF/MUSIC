#include "music/application.hh"

#if MUSIC_USE_MPI
#include <mpi.h>
#include <cassert>

namespace MUSIC
{
    static std::string err_not_runtime = "Application not in running state";
	static std::string err_MPI_Init = "MPI_Init was called before the Setup constructor";

	Application::Application ()
		: Application( std::move (std::make_unique<DefaultContext> ()))
	{}

	Application::Application(std::unique_ptr<MusicContext> context, double timebase)
		: Application(context->getConfiguration(), context->getComm(),
					context->launchedByMusic(), timebase)
	{
		assert (comm_ != MPI::COMM_NULL);
	}

	Application::Application(std::unique_ptr<Configuration> config, MPI::Intracomm comm,
			bool launchedByMusic, double timebase):
		config_ (std::move (config)),
		timebase_ (timebase),
		comm_ (comm),
		launchedByMusic_ (launchedByMusic),
		/* application_map_ (config->applications ()), */
		port_manager_ (*config_),
		runtime_ (nullptr)
	{
	}

	void Application::enterSimulationLoop(double h)
	{
		std::cout << "Entering simulation loop" << std::endl;
		port_manager_.updatePorts ();
		auto ports = port_manager_.getPorts ();
		for (auto& p : ports)
			std::cout << p->name () << " " << std::endl;
		runtime_.reset (new Runtime (*this, ports, h));
		state_ = ApplicationState::RUNNING;
	}

	void Application::exitSimulationLoop()
	{
    	/* MPI::COMM_WORLD.Barrier (); */
		runtime_->finalize ();
		state_ = ApplicationState::STOPPED;
	}

	// TODO all of this repetetive stuff could be realized as template function
	// but then we have to use a single function name for all of them
	// o


	/* std::shared_ptr<ContInputPort> */
	/* Application::publishContInput (std::string identifier) */
	/* { */
	/* 	return port_manager_.createPort<ContInputPort> (*this, identifier); */
	/* } */


	/* std::shared_ptr<ContOutputPort> */
	/* Application::publishContOutput (std::string identifier) */
	/* { */
	/* 	return port_manager_.createPort<ContOutputPort> (*this, identifier); */
	/* } */


	/* std::shared_ptr<EventInputPort> */
	/* Application::publishEventInput (std::string identifier) */
	/* { */
	/* 	return port_manager_.createPort<EventInputPort> (*this, identifier); */
	/* } */


	/* std::shared_ptr<EventOutputPort> */
	/* Application::publishEventOutput (std::string identifier) */
	/* { */
	/* 	return port_manager_.createPort<EventOutputPort> (*this, identifier); */
	/* } */


	/* std::shared_ptr<MessageInputPort> */
	/* Application::publishMessageInput (std::string identifier) */
	/* { */
	/* 	return port_manager_.createPort<MessageInputPort> (*this, identifier); */
	/* } */


	/* std::shared_ptr<MessageOutputPort> */
	/* Application::publishMessageOutput (std::string identifier) */
	/* { */
	/* 	return port_manager_.createPort<MessageOutputPort> (*this, identifier); */
	/* } */

	PortConnectivityManager& Application::getPortConnectivityManager ()
	{
		return port_manager_;
	}

	double Application::timebase() const
	{
		return timebase_;
	}

	int Application::color () const
	{
		return config_-> Color();
	}

	int Application::nProcs () const
	{
		return comm_.Get_size ();
	}

	int Application::leader () const
	{
		return config_->Leader ();
	}

	std::string Application::name () const
	{
		return config_->Name ();
	}

	const ApplicationMap& Application::applicationMap () const
	{
		return *(config_->applications ());
	}

	MPI::Intracomm Application::communicator() const
	{
		return comm_;
	}

	bool Application::launchedByMusic () const
	{
		return launchedByMusic_;
	}

	void Application::finalize()
	{
		// TODO what else?
		if (state_ == ApplicationState::RUNNING)
			exitSimulationLoop ();
		port_manager_.finalize ();
		state_ = ApplicationState::FINALIZED;

		// TODO move to music_context
    	MPI::COMM_WORLD.Barrier ();
		MPI::Finalize ();
	}

	  bool
	  Application::config (std::string var, std::string* result)
	  {
		return config_->lookup (var, result);
	  }


	  bool
	  Application::config (std::string var, int* result)
	  {
		return config_->lookup (var, result);
	  }


	  bool
	  Application::config (std::string var, double* result)
	  {
		return config_->lookup (var, result);
	  }


}

#endif

