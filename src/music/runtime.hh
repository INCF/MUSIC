/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007, 2008, 2009, 2022 INCF
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

#ifndef MUSIC_RUNTIME_HH
//#define MUSIC_DEBUG
#include "music/music-config.hh"
#if MUSIC_USE_MPI
#include <mpi.h>
#include <vector>

#include "music/setup.hh"
#include "music/port.hh"
#include "music/clock.hh"
#include "music/connector.hh"
#include "music/scheduler.hh"
#include "music/scheduler_agent.hh"
namespace MUSIC
{

  /*
   * This is the Runtime object in the MUSIC API
   *
   * It is documented in section 4.4 of the MUSIC manual
   */

  class Runtime
  {
  public:
    Runtime (Setup* s, double h);
    ~Runtime ();

    MPI_Comm
    communicator ();

    void
    finalize ();
    void
    tick ();
    double
    time ();

  private:
    Clock localTime;
    //Clock nextComm;
    std::string app_name;
    int leader_;
    std::vector<std::pair<double, Connector *> > schedule;
    MPI_Comm comm;
    std::vector<Port*> ports;
    std::vector<TickingPort*> tickingPorts;
    std::vector<Connector*> connectors;
    std::vector<PostCommunicationConnector*> postCommunication;
    std::vector<PreCommunicationConnector*> preCommunication;
    Scheduler *scheduler;
    std::vector<SchedulerAgent*> sAgents;
    MulticommAgent* mAgent;
    static bool isInstantiated_;

    typedef std::vector<Connection*> Connections;
    bool needsMultiCommunication ();
    void
    takeTickingPorts (Setup* s);
    void
    connectToPeers (Connections* connections);
    void
    specializeConnectors (Connections* connections);
    void
    spatialNegotiation ();
    void
    takePreCommunicators ();
    void
    takePostCommunicators ();
    void
    buildTables (Setup* s);
    void
    temporalNegotiation (Setup* s, Connections* connections);
    void
    initialize (Setup* s);
  };

}
#endif
#define MUSIC_RUNTIME_HH
#endif
