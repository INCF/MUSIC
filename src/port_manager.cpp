#include "music/port_manager.hh"

#if MUSIC_USE_MPI
#include <mpi.h>

#include "music/runtime.hh"
#include "music/parse.hh"
#include "music/error.hh"
#include "music/application_mapper.hh"
#include <strings.h>
#include <fstream>

namespace MUSIC
{
	static std::string err_no_app_info = "No ApplicationInfo object available"

	const ConnectivityInfo* portConnectivity (const std::string identifier) const
	{
		return connectivityMap_->info (identifier);
	}

	void isInstantiated (std::string identifier)
	{
		auto map_iterator = portMap_.find (identifier);
		if (map_iterator == portMap_.end())
			return false;
		return true;
	}

	void removePort (std::string identifier)
	{
		if (!isInstantiated (identifier))
			error (std::string ("Can not remove port. \
						There is no instance of Port with the given port name."));
		if (isConnected (identifier))
			error (std::string ("Can not remove port. \
						Ports must be disconnected before they can be removed."));
		portMap_.erase (identifier);
	}

	void connect(std::string senderApp, std::string senderPort,
			std::string receiverApp, std::string receiverPort,
			int width,
			ConnectorInfo::CommunicationType commType,
			ConnectorInfo::ProcessingMethod procMethod);
	{
		// To keep the maxPortCode synchron over all MPI processes,
		// this must be executed on all processes (even if they do not handle the
		// requested connection)
		const int portCode = ConnectorInfo::allocPortCode();

		ConnectivityInfo::PortDirection dir;
		ApplicationInfo* remoteInfo;
		if (app_.applicationName () == senderApp)
		{
			// if this app is sender
			dir = ConnectivityInfo::PortDirection::OUTPUT;
			remoteInfo = app_.portConnectivity (receiverApp);
		}
		else if (app_.applicationName () == receiverApp)
		{
			// if this app is receiver
			dir = ConnectivityInfo::PortDirection::INPUT;
			remoteInfo = app_.portConnectivity (senderApp);
		}
		else
		{
			// This connection is not handled by this app. Gracefully return
			return;
		}

		if (remoteInfo == nullptr)
			errorRank(err_no_app_info);

		// where to get the portCode from?
		const int leader = remoteInfo->leader ();

		// TODO does it actually prevent creating the same connection twice?
		connectivityMap_->add (
			dir == ConnectivityInfo::PortDirection::OUTPUT ? senderPort : receiverPort,
			dir, width, receiverApp, receiverPort, portCode, leader,
			remoteInfo->nProc (), commType, procMethod);

	}

	void PortConnectivityManager::disconnect (std::string identifier)
	{
		connectivityMap_.remove (identifier);
	}

	bool PortConnectivityManager::isConnected (std::string identifier) const
	{
		return connectivityMap_->isConnected (identifier);
	}



}
