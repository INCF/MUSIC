#include "music/application.hh"

#if MUSIC_USE_MPI

namespace MUSIC
{
    static std::string err_not_runtime = "Application not in running state";
	static std::string err_MPI_Init = "MPI_Init was called before the Setup constructor";

	void Application::initialize_MPI(int& argc, char**& argv, int required, int* provided)
	{
		// TODO check if this if-block is symantically correct
		if (!MPI::Is_initialized ())
		{
			  /* errorRank (err_MPI_Init); */
#ifdef HAVE_CXX_MPI_INIT_THREAD
			*provided = MPI::Init_thread (argc, argv, required);
#else
			// Only C version provided in libmpich
			MPI_Init_thread (&argc, &argv, required, provided);
		}
#endif
	}

	void Application::initialize_MPI(int& argc, char**& argv)
	{
		if (!MPI::Is_initialized ())
			MPI::Init (argc, argv);
	}

	Application::Application(int& argc, char**& argv,
			double timebase, MUSICLauncher launcher):
		initialize_MPI (argc, argv),
		Application(launcher.getConfiguration(), launcher.getComm(),
				launcher.launchedByMusic(), timebase)
	{
	}

	Application::Application(int& argc, char**& argv, int required, int* provided,
			double timebase, MUSICLauncher launcher):
		initialize_MPI (argc, argv, required, provided),
		Application(launcher.getConfiguration(), launcher.getComm(),
				launcher.launchedByMusic(), timebase)
	{
	}

	Application::Application(std::unique_ptr<Configuration> config, MPI::MPI_Comm comm,
			bool launchedByMusic, double timebase):
		timebase_ (timebase),
		comm_ (comm),
		app_color_ (config->Color ()),
		leader_ (config->Leader ()),
		launchedByMusic_ (launchedByMusic),
		application_map_ (config->applications ()),
		port_manager_ (config->connectivityMap (), *this),
		runtime_(*this, port_manager_, timebase)
	{
	}

	void Application::enterSimulationLoop()
	{

		port_manager_.updatePorts ();
		runtime_ = Runtime(runtime_);
		state_ = ApplicationState::RUNNING;
	}

	void Application::exitSimulationLoop()
	{
		runtime_.finalize ();
		state_ = ApplicationState::STOPPED;
	}

	// TODO all of this repetetive stuff could be realized as template function
	// but then we have to use a single function name for all of them
	std::shared_ptr<ContInputPort>
	Application::publishContInput (std::string identifier)
	{
		return port_manager_.createPort<ContInputPort> (*this, identifier);
	}


	std::shared_ptr<ContOutputPort>
	Application::publishContOutput (std::string identifier)
	{
		return port_manager_.createPort<ContOutputPort> (*this, identifier);
	}


	std::shared_ptr<EventInputPort>
	Application::publishEventInput (std::string identifier)
	{
		return port_manager_.createPort<EventInputPort> (*this, identifier);
	}


	std::shared_ptr<EventOutputPort>
	Application::publishEventOutput (std::string identifier)
	{
		return port_manager_.createPort<EventOutputPort> (*this, identifier);
	}


	std::shared_ptr<MessageInputPort>
	Application::publishMessageInput (std::string identifier)
	{
		return port_manager_.createPort<MessageInputPort> (*this, identifier);
	}


	std::shared_ptr<MessageOutputPort>
	Application::publishMessageOutput (std::string identifier)
	{
		return port_manager_.createPort<MessageOutputPort> (*this, identifier);
	}

	double Application::timebase() const
	{
		return timebase_;
	}

	int Application::nProcs () const
	{
		return comm_.Get_size ();
	}

	int Application::leader () const
	{
		return leader_;
	}

	MPI::Intracomm Application::communicator()
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
		state_ = ApplicationState::FINALIZED;
		MPI::Finalize ();
	}


}

#endif

