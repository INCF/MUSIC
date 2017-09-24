
#ifndef MUSIC_PORT_MANAGER_HH
#include "music/music-config.hh"

#if MUSIC_USE_MPI
#include <map>
#include <string>
#include <memory>
#include <mpi.h>
#include "music/configuration.hh"
#include "music/error.hh"
#include "music/port.hh"
#include "music/misc.hh"


namespace MUSIC
{
	using PortMap = std::map<std::string, std::weak_ptr<Port>>;

	class PortConnectivityManager
	{
		private:
			PortConnectivityManager (Configuration& config)
				  : config_ (config)
				  , portMap_ ()
	              , connectivityModified_ (false)
			{}

			void updatePorts();

		public:

			PortConnectivityManager(const PortConnectivityManager&) = delete;
			PortConnectivityManager& operator= (const PortConnectivityManager&) = delete;


			void connect(std::string senderApp, std::string senderPort,
					std::string receiverApp, std::string receiverPort,
					int width,
					CommunicationType commType,
					ProcessingMethod procMethod);

			void disconnect (std::string appName, std::string portName);
			void disconnect (std::string senderApp, std::string senderPort, std::string receiverApp, std::string receiverPort);

			bool isConnected (std::string identifier) const;
			bool isInstantiated (std::string identifier);

			SPVec<Port> getPorts ();

		private:
			friend class Port;
			friend class Application;

			Configuration& config_;
			PortMap portMap_;
			bool connectivityModified_;

			// TODO move impl to cpp file
			template <typename T>
			void addPort (std::shared_ptr<T> port)
			{
				std::shared_ptr<Port> ptr (std::static_pointer_cast<Port> (port));
				if (isInstantiated (ptr->name ()))
					error (std::string ("Port already has been instantiated."));
				portMap_.emplace (port->name (), port);
			}

			template <typename T>
		    std::shared_ptr<T> createPort (const Application& app, std::string identifier)
			{
				auto ptr (std::make_shared<T> (app, identifier));
				addPort(ptr);
				return ptr;
			}

			void removePort (std::string identifier);
			const ConnectivityInfo& portConnectivity (const std::string localName) const;
			bool isAnyPortExpired () const;
			void finalize ();



	};
}

#endif
#define MUSIC_PORT_MANAGER_HH
#endif

