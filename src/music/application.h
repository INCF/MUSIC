#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <memory>

#include "music/music-config.hh"

#if MUSIC_USE_MPI
#include <mpi.h>
#include "music/configuration.hh"
#include "music/error.hh"
#include "music/port.hh"
#include "music/runtime.hh"


namespace MUSIC
{

	class enum ApplicationState {RUNNING, STOPPED, FINALIZED};
	using Ports = std::vector<std::shared_ptr<Port>>;
	using PortMap = std::map<std::string, std::weak_ptr<Port>>;


	class Application
	{
		public:
			Application(Configuration config, double h);
			/* Application(Configuration config, double h, MPI::MPI_Comm comm); */

			Application(const Application&) = delete;
			Application& operator= (const Application&) = delete;

			// TODO move and move-assignment constructors

			void tick();
			void enterSimulationLoop();
			void exitSimulationLoop();
			void finalize();

			double time() const;
			double timebase() const;
			std::string applicationName() const;
			Ports getConnectedPorts() const;
			bool launchedByMusic () const;


			// I really dont like this solution but for now it costs less time
			// than refactoring the whole Configuration business
			void setConfiguration(Configuration config);


			// Why pointer?
			std::shared_ptr<ContInputPort> publishContInput (std::string identifier);
			std::shared_ptr<ContOutputPort> publishContOutput (std::string identifier);
			std::shared_ptr<EventInputPort> publishEventInput (std::string identifier);
			std::shared_ptr<EventOutputPort> publishEventOutput (std::string identifier);
			std::shared_ptr<MessageInputPort> publishMessageInput (std::string identifier);
			std::shared_ptr<MessageOutputPort> publishMessageOutput (std::string identifier);

		private:
			/* void init(); */
			Configuration conf_;
			double h_;
			MPI::MPI_Comm comm_ {MPI::COMM_WORLD};
			ApplicationState state_ {ApplicationState::STOPPED};
			PortMap port_map_;
			bool launchedByMusic_ {false};

		private:
			friend class Port;

			std::unique_ptr<Runtime> runtime_:
			void assertValidState(std::string func_name, ApplicationState as);
			void checkConnectedPortMissing(const Configuration& c) const;
			bool isPortConnected(std::string identifier) const;
			/* void addPort(Port* p); */
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

	inline void Application::finalize()
	{
		runtime_->finalize();
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
