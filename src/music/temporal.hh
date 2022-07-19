/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2008, 2009, 2022 INCF
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

#ifndef MUSIC_TEMPORAL_HH
#include "music/music-config.hh"
#if MUSIC_USE_MPI
#define MAX_BUFFERED_NO_VALUE -1
#define DEFAULT_PACKET_SIZE 64000
#define EVENT_FREQUENCY_ESTIMATE 10.0
#define DEFAULT_MESSAGE_MAX_BUFFERED 10
#include <music/debug.hh>
#include <music/clock.hh>
#include <music/connection.hh>
#include <music/application_graph.hh>
#include "music/error.hh"
namespace MUSIC
{

  class Setup;

  class ConnectionDescriptor
  {
  public:
    int remoteNode;
    int receiverPort;
    int maxBuffered;
    int defaultMaxBuffered; // not used for input connections
    bool interpolate;
    bool multiComm;
    ClockState accLatency;
    ClockState remoteTickInterval;
  };


  class TemporalNegotiationData
  {
  public:
    int color;
    int leader;
    int nProcs;
    double timebase;
    ClockState tickInterval;
    int nOutConnections;
    int nInConnections;
    ConnectionDescriptor connection[1];
  };


  class TemporalNegotiatorGraph : public AGraph<TemporalNegotiationData,
      ConnectionDescriptor>
  {
    double timebase_;
    int nConnections_;
  public:
    TemporalNegotiatorGraph (double timebase, int nApps, int color) :
        AGraph<TemporalNegotiationData, ConnectionDescriptor> (color, nApps), timebase_ (
            timebase)
    {
    }

    void
    setNConnections (int nConns);
    int
    getNConnections ();
  protected:
    AEdge<TemporalNegotiationData, ConnectionDescriptor>
    edge (ANode<TemporalNegotiationData, ConnectionDescriptor> &x, int c)
    {
      ConnectionDescriptor& descr = x.data ().connection[c];

      return AEdge<TemporalNegotiationData, ConnectionDescriptor> (x,
          nodes_[descr.remoteNode], descr);
    }

    void
    handleLoop (ANode<TemporalNegotiationData, ConnectionDescriptor> &x,
        std::vector<AEdge<TemporalNegotiationData, ConnectionDescriptor> > & path);
  };

  // The TemporalNegotiator negotiates communication timing parameters
  // with all other applications.
  class TemporalNegotiator
  {
    Setup* setup_;
    MPI_Group groupWorld;
    MPI_Group applicationLeaders;
    MPI_Comm negotiationComm;

    int nApplications_; // initialized by createNegotiationCommunicator
    int nLocalConnections;
    int nAllConnections;

    std::vector<OutputConnection> outputConnections;
    std::vector<InputConnection> inputConnections;

    TemporalNegotiatorGraph *nodes;

    TemporalNegotiationData* negotiationBuffer;
    TemporalNegotiationData* negotiationData;

  public:
    TemporalNegotiator (Setup* setup);
    ~TemporalNegotiator ();
    void
    negotiate (Clock& localTime, std::vector<Connection*>* connections);
    TemporalNegotiatorGraph*
    applicationGraph ();

  private:
    void
    separateConnections (std::vector<Connection*>* connections);
    void
    createNegotiationCommunicator ();
    void
    collectNegotiationData (ClockState ti);
    void
    communicateNegotiationData ();
    void
    combineParameters ();
    void
    loopAlgorithm ();
    void
    distributeParameters ();
    void
    broadcastNegotiationData ();
    void
    receiveNegotiationData ();
    void
    fillTemporalNegotiatorGraph ();
    int
    negotiationDataSize (int nConnections);
    int
    negotiationDataSize (int nBlock, int nConnections);
    int
    computeDefaultMaxBuffered (int maxLocalWidth, int eventSize,
        ClockState tickInterval, double timebase);
    TemporalNegotiationData*
    allocNegotiationData (int nBlocks, int nConnections);
    void
    freeNegotiationData (TemporalNegotiationData*);
    int
    findNodeColor (int leader);
    ConnectionDescriptor*
    findInputConnection (int node, int port);
    bool
    isLeader ();
    bool
    hasPeers ();
  };
}
#endif
#define MUSIC_TEMPORAL_HH
#endif
