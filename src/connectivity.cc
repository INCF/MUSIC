/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2008, 2009, 2012 INCF
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

#include <algorithm>

#include "music/music-config.hh"

#include "music/debug.hh"

#include "music/connectivity.hh"
#include "music/ioutils.hh"
#include "music/error.hh"

namespace MUSIC {

  using CMapIterator = std::map<std::string, ConnectivityInfo>::iterator;
  int ConnectorInfo::maxPortCode_;

#if __cplusplus <=199711L
  ConnectivityInfo const Connectivity::NO_CONNECTIVITY = NULL;
#endif

  void
  ConnectorInfo::registerPortCode (int portCode)
  {
    ConnectorInfo::maxPortCode_ = std::max (portCode,
					    ConnectorInfo::maxPortCode_);
  }


  void
  ConnectivityInfo::addConnection (std::string recApp,
				   std::string recName,
				   int recCode,
				   int rLeader,
				   int nProc,
				   int commType,
				   int procMethod
					)
  {
    portConnections_.push_back (ConnectorInfo (recApp,
					       recName,
					       recCode,
					       rLeader,
					       nProc,
					       commType,
					       procMethod));
  }

  // TODO assymetric right now, fix it!
  void
  ConnectivityInfo::removeConnection (std::string recApp, std::string recName)
  {
	// iterate over all connectorInfos and remove the corresponding ones
		// TODO reconsider using a std::list instead of vector. Removing
		// from vector is really expensive
		portConnections_.erase(
				std::remove_if(portConnections_.begin(), portConnections_.end(),
					   [&](ConnectorInfo& ci) {
					   return (ci.receiverAppName() == recApp
							   && ci.receiverPortName() == recName);
					   }), portConnections_.end());

  }


  void
  Connectivity::add (std::string localPort,
		     ConnectivityInfo::PortDirection dir,
		     int width,
		     std::string recApp,
		     std::string recPort,
		     int recPortCode,
		     int remoteLeader,
		     int remoteNProc,
		     int commType,
			 int procMethod
				)
  {
    CMapIterator cmapInfo
      = connectivityMap.find (localPort);
	ConnectivityInfo info;
    if (cmapInfo == connectivityMap.end ())
      {
		MUSIC_LOG ("creating new entry for " << localPort);
		connectivityMap.insert (
				std::make_pair {localPort,
					ConnectivityInfo (localPort, dir, width)});
		info = connectivityMap.find (localPort)->second;

      }
    else
      {
		MUSIC_LOG ("found old entry for " << localPort);
		/* info = &connections_[cmapInfo->second]; */
		info = cmapInfo->second;
		if (info->direction () != dir)
		  error ("port " + localPort + " used both as output and input");
      }
    info->addConnection (recApp,
			 recPort,
			 recPortCode,
			 remoteLeader,
			 remoteNProc,
			 commType,
			 procMethod
				);
  }

  // TODO assymetric right now, fix it!
  void
  Connectivity::remove (std::string localPort, std::string recApp, std::string recPort)
  {
	auto cmapInfo = connectivityMap.find (localPort);
	if (cmapInfo == connectivityMap.end ())
		return;
	ConnectivityInfo* info = cmapInfo->second;
	info->removeConnection (recApp, recPort);
	if (info->connections ().empty())
		remove (localPort);

  }

  void
  ConnectivityInfo::remove (std::string localPort)
  {
	auto cmapInfo = connectivityMap.find (localPort);
	if (cmapInfo == connectivityMap.end ())
		return;
	connectivityMap.erase (localPort);
  }


  ConnectivityInfo
  Connectivity::info (std::string portName)
  {
    CMapIterator info
      = connectivityMap.find (portName);
    if (info == connectivityMap.end ())
      return NO_CONNECTIVITY;
    else
	  return info->second;
  }


  bool
  Connectivity::isConnected (std::string portName)
  {
    return connectivityMap.find (portName) != connectivityMap.end ();
  }


  ConnectivityInfo::PortDirection
  Connectivity::direction (std::string portName)
  {
    return connectivityMap[portName]->direction ();
  }


  int
  Connectivity::width (std::string portName)
  {
    return connectivityMap[portName]->width ();
  }


  // TODO as copy or share ptr?
  PortConnectorInfo
  Connectivity::connections (std::string portName)
  {
    return connectivityMap[portName]->connections ();
  }


  void
  Connectivity::write (std::ostringstream& out)
  {
    out << connectivityMap.size ();
    CMapIterator i;
    for (i = connectivityMap.begin ();
	 i != connectivityMap.end ();
	 ++i)
      {
	out << ':' << i->first << ':';
	ConnectivityInfo ci = i->second;
	out << ci->direction () << ':' << ci->width () << ':';
	PortConnectorInfo conns = ci->connections ();
	out << conns.size ();
	PortConnectorInfo::iterator c;
	for (c = conns.begin (); c != conns.end (); ++c)
	  {
	    out << ':' << c->receiverAppName ();
	    out << ':' << c->receiverPortName ();
	    out << ':' << c->receiverPortCode ();
	    out << ':' << c->remoteLeader ();
	    out << ':' << c->nProcesses ();
	    out << ':' << c->communicationType();
	    out << ':' << c->processingMethod();
	  }
      }
  }


  void
  Connectivity::read (std::istringstream& in, std::map<int, int> leaders)
  {
    int nPorts;
    in >> nPorts;
    for (int i = 0; i < nPorts; ++i)
      {
	in.ignore ();
	std::string portName = IOUtils::read (in);
	in.ignore ();
	int dir;
	in >> dir;
	ConnectivityInfo::PortDirection pdir
	  = static_cast<ConnectivityInfo::PortDirection> (dir);
	in.ignore ();
	int width;
	in >> width;
	in.ignore ();
	int nConnections;
	in >> nConnections;
	for (int i = 0; i < nConnections; ++i)
	  {
	    in.ignore ();
	    std::string recApp = IOUtils::read (in);
	    in.ignore ();
	    std::string recPort = IOUtils::read (in);
	    in.ignore ();
	    int recPortCode;
	    in >> recPortCode;
	    ConnectorInfo::registerPortCode (recPortCode);
	    in.ignore ();
	    // leader information is not available through configuration string
	    // application color is used instead
	    int color;
	    in >> color;
	    in.ignore ();
	    int nProc;
	    in >> nProc;
	    in.ignore ();
	    int commType;
	    in >> commType;
	    in.ignore ();
	    int procMethod;
	    in >> procMethod;
	    add (portName,
		 pdir,
		 width,
		 recApp,
		 recPort,
		 recPortCode,
		 leaders[color],
		 nProc,
		 commType,
		 procMethod
		 );
	    MUSIC_LOG ("add (portName = " << portName
		       << ", pdir = " << pdir
		       << ", width = " << width
		       << ", recApp = " << recApp
		       << ", recPort = " << recPort
		       << ", rLeader = " << leaders[color]
		       << ", nProc = " << nProc
		       << ", commType = " << commType
		       << ", procMethod = " << procMethod
		       << ")");
	  }
      }
  }


}
