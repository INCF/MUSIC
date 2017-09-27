/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2017 INCF
 *
 *  MUSIC is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  MUSIC is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "music/port_manager.hh"

#if MUSIC_USE_MPI
#include <mpi.h>

#include "music/parse.hh"
#include "music/application_mapper.hh"
#include <strings.h>
#include <fstream>
#include "music/error.hh"

namespace MUSIC
{
  static std::string err_no_app_info = "No ApplicationInfo object available";

  const ConnectivityInfo& PortConnectivityManager::portConnectivity (const std::string identifier) const
  {
    auto info = config_.connectivityMap ()->info (identifier);
    if (info == nullptr)
      error ("No connectivity info for port " + identifier + " found");
    return *info;
  }

  bool PortConnectivityManager::isInstantiated (std::string identifier)
  {
    auto map_iterator = portMap_.find (identifier);
    if (map_iterator == portMap_.end() || map_iterator->second.expired ())
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

  bool PortConnectivityManager::isAnyPortExpired () const
  {
    for (auto& p : portMap_)
      {
	if (p.second.expired ())
	  return true;
	return false;
      }
  }

  void PortConnectivityManager::updatePorts ()
  {
    // only perform expensive updates when necessary
    /* if (connectivityModified_) */
    /* { */
    std::cout << "Reconneting ports..." << std::endl;
    for (auto& port : getPorts ())
      port->reconnect ();
    connectivityModified_ = false;
    std::cout << "Reconnecting done." << std::endl;
    /* } */
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
					 CommunicationType commType,
					 ProcessingMethod procMethod)
  {
    // To keep the maxPortCode synchron over all MPI processes,
    // this must be executed on all processes (even if they do not handle the
    // requested connection)
    int portCode = ConnectorInfo::allocPortCode();

    ConnectivityInfo::PortDirection dir;
    const ApplicationInfo* remoteInfo;
    if (config_.Name ()== senderApp)
      {
	// if this app is sender
	dir = ConnectivityInfo::PortDirection::OUTPUT;
	remoteInfo = config_.applications ()->lookup (receiverApp);
      }
    else if (config_.Name() == receiverApp)
      {
	// if this app is receiver
	dir = ConnectivityInfo::PortDirection::INPUT;
	remoteInfo = config_.applications ()->lookup (senderApp);
      }
    else
      {
	// This connection is not handled by this app. Gracefully return
	return;
      }

    if (remoteInfo == nullptr)
      errorRank(err_no_app_info);

    // where to get the portCode from?
    int leader = remoteInfo->leader ();

    // TODO does it actually prevent creating the same connection twice?
    config_.connectivityMap()->add (
				    dir == ConnectivityInfo::PortDirection::OUTPUT ? senderPort : receiverPort,
				    dir, width, receiverApp, receiverPort, portCode, leader,
				    remoteInfo->nProc (), commType, procMethod);
    connectivityModified_ = true;

  }

  void PortConnectivityManager::disconnect (std::string appName, std::string portName)
  {
    if (config_.Name ()== appName)
      config_.connectivityMap ()->remove (portName);
    else
      {
	auto connectedPorts = config_.connectivityMap ()->getConnectedLocalPorts (portName, appName);
	for (auto& localPort : connectedPorts)
	  config_. connectivityMap ()->remove (localPort, appName, portName);
      }
    connectivityModified_ = true;
  }

  void PortConnectivityManager::disconnect (std::string senderApp, std::string senderPort, std::string receiverApp, std::string receiverPort)
  {
    if (config_. Name ()== senderApp)
      config_. connectivityMap ()->remove (senderPort, receiverApp, receiverPort);
    else if (config_. Name ()== receiverApp)
      config_. connectivityMap ()->remove (receiverPort, senderApp, senderPort);
    else
      return;
    connectivityModified_ = true;
  }

  bool PortConnectivityManager::isConnected (std::string identifier) const
  {
    return config_. connectivityMap ()->isConnected (identifier);
  }

  void PortConnectivityManager::finalize ()
  {
    for (auto& p : getPorts ())
      {
	p->removeConnections ();
      }
  }
}
#endif
