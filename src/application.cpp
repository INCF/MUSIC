#include "music/application.hh"
#if MUSIC_USE_MPI

namespace MUSIC
{
    static std::string err_not_runtime = "Application not in running state";

	Application::Application(Configuration config, double h):
		conf_(config), h_(h), runtime_(nullptr)
	{
	}

	Application::Application(Configuration config, double h, MPI::MPI_Comm comm):
		Application(config, h), comm_(comm)
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
		return h_;
	}

	MPI::Intracomm Application::communicator()
	{
		return comm_;
	}

	void Application::setConfiguration(Configuration config)
	{
		if (state_ != ApplicationState::STOPPED)
			errorRank(std::string("Setting new Configuration outside STOPPED state is illegal"));
		if (!config.Name().compare(conf_.Name()))
		{
			std::stringstream ss;
			ss << "Application name specified in the new Configuration object does not match the name of this Application";
			errorRank(ss.str());
		}
		checkConnectedPortMissing(config);
		conf_ = config;
	}

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
		for ()
	}

	bool Application::launchedByMusic () const
	{
		return launchedByMusic_;
	}

}

#endif

