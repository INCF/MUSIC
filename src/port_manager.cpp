#include "music/port_manager.hh"

#if MUSIC_USE_MPI
#include <mpi.h>

#include "music/parse.hh"
#include "music/application_mapper.hh"
#include <strings.h>
#include <fstream>

namespace MUSIC
{
	static std::string err_no_app_info = "No ApplicationInfo object available";

	const ConnectivityInfo* PortConnectivityManager::portConnectivity (const std::string identifier) const
	{
		return connectivityMap_->info (identifier);
	}

	bool PortConnectivityManager::isInstantiated (std::string identifier)
	{
		auto map_iterator = portMap_.find (identifier);
		if (map_iterator == portMap_.end())
			return false;
		return true;
	}

	SPVec<Port> PortConnectivityManager::getPorts ()
	{
		SPVec<Port> v;
		for (auto& p : portMap_)
		{
			auto spt = p.second.lock ();
			if (spt)
				v.push_back (spt);
		}
		return v;
	}

	void PortConnectivityManager::removePort (std::string identifier)
	{
		if (!isInstantiated (identifier))
			error (std::string ("Can not remove port. \
						There is no instance of Port with the given port name."));
		if (isConnected (identifier))
			error (std::string ("Can not remove port. \
						Ports must be disconnected before they can be removed."));
		portMap_.erase (identifier);
	}

	void PortConnectivityManager::connect (std::string senderApp, std::string senderPort,
			std::string receiverApp, std::string receiverPort,
			int width,
			ConnectorInfo::CommunicationType commType,
			ConnectorInfo::ProcessingMethod procMethod)
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

	void PortConnectivityManager::disconnect (std::string appName, std::string portName)
	{
		if (app_.applicationName () == appName)
			connectivityMap_.remove (portName);
		else
		{
			auto connectedPorts = connectivityMap_.getConnectedLocalPorts (portName, appName);
			for (auto& localPort : connectedPorts)
				connectivityMap_.remove (localPort, appName, portName);
		}

	}

	void PortConnectivityManager::disconnect (std::string senderApp, std::string senderPort, std::string receiverApp, std::string receiverPort)
	{
		if (app_.applicationName () == senderApp)
			connectivityMap_.remove (senderPort, recApp, recPort);
		else if (app_.applicationName () == receiverApp)
			connectivityMap_.remove (receiverPort, senderApp, senderPort);
	}

	bool PortConnectivityManager::isConnected (std::string identifier) const
	{
		return connectivityMap_->isConnected (identifier);
	}



}
