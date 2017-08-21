#include "music/application.hh"
#if MUSIC_USE_MPI

namespace MUSIC
{
    static std::string err_not_runtime = "Application not in running state";
	static std::string err_MPI_Init = "MPI_Init was called before the Setup constructor";

	void initialize_MPI(int& argc, char**& argv, int required, int* provided)
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

	void initialize_MPI(int& argc, char**& argv)
	{
		// TODO check if auto-init is desireable
		if (!MPI::Is_initialized ())
		  /* errorRank (err_MPI_Init); */
			MPI::Init (argc, argv);
	}

	// TODO rewrite constructors
	Application::Application(int& argc, char**& argv, int required, int* provided, double timebase, MPI::MPI_Comm comm):
		initialize_MPI (argc, argv, required, provided),
	{
	}

	// TODO Maybe overload with move constructor?
	Application::Application(Configuration config, double timebase, MPI::MPI_Comm comm):
		conf_(config), timebase_(timebase), comm_(comm), runtime_(nullptr)
	{
	}

	void Application::enterSimulationLoop()
	{
		state_ = ApplicationState::RUNNING;
		// Eventually run negotiators etc. again
	}

	void Application::exitSimulationLoop()
	{
		state_ = ApplicationState::STOPPED;
	}

	std::shared_ptr<ContInputPort>
	Application::publishContInput (std::string identifier)
	{
	return std::make_shared<ContInputPort> (&this, identifier);
	}


	std::shared_ptr<ContOutputPort>
	Application::publishContOutput (std::string identifier)
	{
	return std::make_shared<ContOutputPort> (&this, identifier);
	}


	std::shared_ptr<EventInputPort>
	Application::publishEventInput (std::string identifier)
	{
	return std::make_shared<EventInputPort> (&this, identifier);
	}


	std::shared_ptr<EventOutputPort>
	Application::publishEventOutput (std::string identifier)
	{
	return std::make_shared<EventOutputPort> (&this, identifier);
	}


	std::shared_ptr<MessageInputPort>
	Application::publishMessageInput (std::string identifier)
	{
	return std::make_shared<MessageInputPort> (&this, identifier);
	}


	std::shared_ptr<MessageOutputPort>
	Application::publishMessageOutput (std::string identifier)
	{
	return std::make_shared<MessageOutputPort> (&this, identifier);
	}

	/* void Application::addPort(Port* p) */
	/* { */
	/* 	ports_.push_back(p); */
	/* } */

	double Application::timebase() const
	{
		return timebase_;
	}

	MPI::Intracomm Application::communicator()
	{
		return comm_;
	}

	/* void Application::setConfiguration(Configuration config) */
	/* { */
	/* 	if (state_ != ApplicationState::STOPPED) */
	/* 		errorRank(std::string("Setting new Configuration outside STOPPED state is illegal")); */
	/* 	if (!config.Name().compare(conf_.Name())) */
	/* 	{ */
	/* 		std::stringstream ss; */
	/* 		ss << "Application name specified in the new Configuration object does not match the name of this Application"; */
	/* 		errorRank(ss.str()); */
	/* 	} */
	/* 	checkConnectedPortMissing(config); */
	/* 	conf_ = config; */
	/* } */

	bool Application::isPortConnected(std::string identifier) const
	{
		std::weak_ptr<Port> wp = port_map_[identifier];
		if (auto sp = wp.lock())
			return sp->isConnected();
		else
			return false;
	}

	void Application::checkConnectedPortMissing(const Configuration& c) const
	{
		Connectivity* connectivity = c.connectivityMap();
		for ( auto& e : port_map_ )
		{
			std::string port_name = e.first;
			// Port with name 'port_name' does not exist in new config??
			if (isPortConnected(port_name) &&
					(connectivity->info(port_name) == Connectivity::NO_CONNECTIVITY))
			{
				std::stringstream ss;
				ss << "Bla";
				errorRank(ss.str());
			}

		}
	}

	Ports Application::getConnectedPorts() const
	{
		// TODO
	/* for () */
	}

	bool Application::launchedByMusic () const
	{
		return launchedByMusic_;
	}

	void Application::connect(
			std::string senderApp, std::string senderPort,
			std::string receiverApp, std::string receiverPort,
			const int width,
			ConnectorInfo::CommunicationType commType,
			ConnectorInfo::ProcessingMethod procMethod)
	{
		// TODO
		// check if Port is ready to connect

		// Copy-Pasta from application_mapper
		ConnectivityInfo::PortDirection dir;
		ApplicationInfo* remoteInfo;
		if (conf_.Name () == senderApp)
		{
			// if this app is sender
			dir = ConnectivityInfo::PortDirection::OUTPUT;
			remoteInfo = conf_.applications()->lookup (receiverApp);
		}
		else if (config_.Name () == receiverApp)
		{
			// if this app is receiver
			dir = ConnectivityInfo::PortDirection::INPUT;
			remoteInfo = conf_.applications()->lookup (senderApp);
		}
		else
		{
			return;
		}

		// where to get the portCode from?
		int leader = remoteInfo->leader ();
		conf_.connectivityMap()->add (
			dir == ConnectivityInfo::PortDirection::OUTPUT ? senderPort : receiverPort,
			dir, width, receiverApp, receiverPort, portCode, leader,
			remoteInfo->nProc (), commType, procMethod);
	}


}

#endif

