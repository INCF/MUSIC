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

#ifndef MUSIC_CONNECTIVITY_HH

#include <sstream>
#include <vector>
#include <map>
#include <list>
#include <limits>

#include "music/error.hh"

namespace MUSIC {
/*
 * Communication Type (<COLLECTIVE, POINTTOPOINT>)
 * Processing Method (<TREE, TABLE>)
 */
	// TODO change back to enum class ... I sacraficed it here to the sake of
	// stupid Cython that has no native support for enum class types
    enum CommunicationType : int
    {
      COLLECTIVE, POINTTOPOINT
    };


    enum ProcessingMethod : int
    {
      TREE, TABLE
    };

  class ConnectorInfo
  {
  public:

    static int maxPortCode_;
    std::string recApp_;
    std::string recPort_;
    int recCode_;
    int remoteLeader_;
    int nProc_;
    CommunicationType commType_;
    ProcessingMethod procMethod_;

  public:
    ConnectorInfo ()
    {
    }


    ConnectorInfo (std::string recApp, std::string recName, int recCode,
        int rLeader, int nProc, CommunicationType commType, ProcessingMethod procMethod) :
        recApp_ (recApp), recPort_ (recName), recCode_ (recCode), remoteLeader_ (
            rLeader), nProc_ (nProc), commType_ (commType), procMethod_ (
            procMethod)
    {
    }


    static void
    registerPortCode (int portCode);


    static int
    allocPortCode ()
    {
	   // TODO emit error msg
	  if (maxPortCode_  == std::numeric_limits<int>::max() )
		  error (std::string ("Maximum number of ports allocated. Aborting."));
      return ++maxPortCode_;
    }


    std::string
    receiverAppName () const
    {
      return recApp_;
    }


    std::string
    receiverPortName () const
    {
      return recPort_;
    }


    int
    receiverPortCode () const
    {
      return recCode_;
    }


    int
    remoteLeader () const
    {
      return remoteLeader_;
    }


    void
    setRemoteLeader (int leader)
    {
      remoteLeader_ = leader;
    }


    // NOTE: nProcesses should have "remote" in name
    int
    nProcesses () const
    {
      return nProc_;
    }


    CommunicationType
    communicationType () const
    {
      return commType_;
    }


    ProcessingMethod
    processingMethod () const
    {
      return procMethod_;
    }
  };


  typedef std::vector<ConnectorInfo> PortConnectorInfo;

  class ConnectivityInfo
  {

  public:
    enum class PortDirection : int
    {
      OUTPUT, INPUT
    };



    static const int NO_WIDTH = -1;

  private:
    std::string portName_;
    PortDirection dir_;
    int width_;
    PortConnectorInfo portConnections_;

  public:
    ConnectivityInfo (std::string portName, PortDirection dir, int width) :
        portName_ (portName), dir_ (dir), width_ (width)
    {
    }


    std::string
    portName ()
    {
      return portName_;
    }


    PortDirection direction()
    {
      return dir_;
    }


    int
    width ()
    {
      return width_;
    } // NO_WIDTH if no width specified


    PortConnectorInfo&
    connections ()
    {
      return portConnections_;
	  /* PortConnectorInfo tmp; */
	  /* std::transform(connectivityMap.begin(), connectivityMap.end(), */
			  /* std::back_inserter(tmp), */
			  /* [](auto& e) {return e.second;}); */
	  /* return tmp; */
    }


    void addConnection (std::string recApp, std::string recName, int recCode,
        int rLeader, int nProc, CommunicationType commType, ProcessingMethod procMethod);

	void removeConnection (std::string recApp, std::string recName);

  };




  class Connectivity
  {
    /* std::vector<ConnectivityInfo> connections_; */
    std::map<std::string, ConnectivityInfo*> connectivityMap;

  public:
    Connectivity ()
    {
    }

	~Connectivity ()
	{
		for (auto& e : connectivityMap)
		{
			delete e.second;
		}
	}

/* #if __cplusplus > 199711L */
	static constexpr ConnectivityInfo* NO_CONNECTIVITY = nullptr;
/* #else */
/*     static ConnectivityInfo* const NO_CONNECTIVITY; */
/* #endif */

    void  add (std::string localPort, ConnectivityInfo::PortDirection dir, int width,
        std::string recApp, std::string recPort, int recPortCode,
        int remoteLeader, int remoteNProc, CommunicationType commType, ProcessingMethod procMethod);

	void remove (std::string localPort, std::string recApp, std::string recPort);

	void remove (std::string localPort);

	ConnectivityInfo* info (std::string portName);


    bool isConnected (std::string portName);

    ConnectivityInfo::PortDirection direction (std::string portName);

    int width (std::string portName);

    PortConnectorInfo connections (std::string portName);

    void write (std::ostringstream& out);

    void read (std::istringstream& in, std::map<int, int> leaders);

    std::list<std::string> getConnectedLocalPorts (std::string remotePort, std::string remoteApp);

  };

}

#define MUSIC_CONNECTIVITY_HH
#endif
