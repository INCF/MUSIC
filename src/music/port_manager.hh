
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

namespace MUSIC
{
	// We could also use raw pointers but we do not (see application.hh why so)
	using PortMap = std::map<std::string, std::weak_ptr<Port>>;

	class PortConnectivityManager
	{
		public:
			PortConnectivityManager ()
				: connectivityMap_ (std::make_unique<Connectivity> ()),
				app_ () {}

			PortConnectivityManager (std::unique_ptr<Connectivity> connectivityMap,
					Application& app)
				: connectivityMap_ (connectivityMap), portMap_ (), app_ (app) {}

			PortConnectivityManager(const PortConnectivityManager&) = delete;
			PortConnectivityManager& operator= (const PortConnectivityManager&) = delete;


			void connect(std::string senderApp, std::string senderPort,
					std::string receiverApp, std::string receiverPort,
					int width,
					ConnectorInfo::CommunicationType commType,
					ConnectorInfo::ProcessingMethod procMethod);

			void disconnect (std::string identifier);
			void disconnect (std::shared_ptr<Port> port);

			bool isConnected (std::string identifier) const;
			bool isInstantiated (std::string identifier);

		private:
			friend class Port;

			template <typename T>
			void addPort (std::shared_ptr<T> port);
			{
				std::shared_ptr<Port> ptr (std::static_pointer_cast<Port> (port));
				if (isInstantiated (ptr->name ()))
					error (std::string ("Port already has been instantiated."));
				portMap_.emplace (port->name (), port);
			}

			void removePort (std::string identifier);
			const ConnectivityInfo* portConnectivity (const std::string localName) const;

			std::unique_ptr<Connectivity> connectivityMap_;
			PortMap portMap_;
			Application& app_;


	};
}

#endif
#define MUSIC_PORT_MANAGER_HH
#endif

