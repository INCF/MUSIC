/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2011, 2012, 2022 INCF
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

#ifndef MUSIC_SCHEDULER_HH

#include "music/music-config.hh"

#if MUSIC_USE_MPI
#include <limits>
#include <vector>
#include <music/clock.hh>
#include <music/connector.hh>
#include <music/multibuffer.hh>
#include <music/application_map.hh>
#include <music/temporal.hh>
namespace MUSIC
{

// The Scheduler is responsible for the timing involved in
// communication, sampling, interpolation and buffering.
  class SchedulerAgent;
  class SchedulerAgent;
  class SApplData;
  class SConnData;

  class Scheduler
  {

  public:

    class SConnData;


    class SApplData
    {
    public:
      Clock localTime;
      int leader;
      int nProcs;
      int color;
    };


    class SConnData
    {
    public:
      Clock nextSend;
      Clock nextReceive;
      Clock sSend;
      ConnectionDescriptor descr;
      Connector *connector;
      bool isLoopConnected;


      void
      reset ()
      {
        nextSend.reset ();
        nextReceive.reset ();
        sSend.reset ();
        isLoopConnected = false;
      }


      void
      postponeNextSend (Clock newTime)
      {
        sSend = nextSend;
        nextSend = newTime;
      }
    };


    typedef ANode<SApplData, SConnData> SNode;
    typedef AEdge<SApplData, SConnData> SConnection;


    class SGraph : public AGraph<SApplData, SConnData>
    {
      int tmp_color_;
    public:
      SGraph (int color, int nApps, int nEdges) :
          AGraph<SApplData, SConnData> (color, nApps, nEdges), tmp_color_ (
              color)
      {
      }
      ;


      void
      setTempColor (int tmp_color)
      {
        tmp_color_ = tmp_color;
      }


      void handleLoop (SNode &x, std::vector<SConnection> & path);
    };


  private:
    SGraph *nodes;
    SGraph::iterator iter_node;
    SNode::edge_iterator iter_conn;

    std::vector<SchedulerAgent *> agents_;
    std::vector<SchedulerAgent *>::iterator cur_agent_;

    SConnection last_sconn_;

    SApplData *appl_data_;
    SConnData *conn_data_;

    int color_;
    int leader_;
    MPI_Comm comm_;
  public:

    Scheduler (MPI_Comm comm, int leader);
    ~Scheduler ();

    void initialize (TemporalNegotiatorGraph *appl_graph,
        std::vector<Connector*>& connectors);

    void finalize (Clock& localTime, std::vector<Connector*>& connectors);

    void reset (int self_node);

#if 0
    void createMultiConnectors (Clock localTime,
        std::vector<Connector*>& connectors,
        MultiBuffer* multiBuffer,
        std::vector<MultiConnector*>& multiConnectors);
    void createMultiConnNext (int self_node,
        Clock& localTime,
        std::vector<Connector*>& connectors,
        MultiBuffer* multiBuffer,
        std::vector<std::pair<double, Connector *> > &schedule);
    void createMultiConnStep (int self_node,
        Clock& localTime,
        std::vector<Connector*>& connectors,
        MultiBuffer* multiBuffer,
        std::vector<MultiConnector*>& multiConnectors,
        std::vector<bool>& multiProxies,
        std::vector<Connector*>& cCache,
        std::vector<std::pair<double, Connector *> > &schedule,
        bool finalize);
    void nextCommunication (Clock& localTime,
        std::vector<std::pair<double, Connector *> > &schedule);

#endif
    void setAgent (SchedulerAgent* agent);

    void initializeAgentState ();

    AEdge<Scheduler::SApplData, Scheduler::SConnData>
    pushForward ();

    void tick (Clock& localTime);

    bool
    isLocalConnection (SConnection &edge)
    {

      return &edge.pre () == &nodes->at (color_)
          || &edge.post () == &nodes->at (color_);

    }

    unsigned int
    nApplications ()
    {
      return nodes->nNodes ();
    }

    int
    self_node ()
    {
      return color_;
    }

    SConnection &
    getLastSConnection ()
    {
      return last_sconn_;
    }

    int
    getLeader ()
    {
      return leader_;
    }

    MPI_Comm
    comm ()
    {
      return comm_;
    }
  private:

    void reset ();

    void setApplications (TemporalNegotiatorGraph *appl_graph);

    void setConnections (TemporalNegotiatorGraph *appl_graph,
        std::vector<Connector*>& connectors);

    Clock nextApplicationReceive (SNode &node);

    void advanceConnection (SConnData &data);

    void advanceConnectionClocks (SConnData &data);
  };

}
#endif

#define MUSIC_SCHEDULER_HH
#endif
