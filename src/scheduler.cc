/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2012, 2022 INCF
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

//#define MUSIC_DEBUG
#include "music/scheduler.hh"
#include "music/scheduler_agent.hh"

#include "music/debug.hh"

#if MUSIC_USE_MPI

#include <cmath>
#include <iostream>
#include <set>
#include <climits>

#ifdef MUSIC_DEBUG
#include <cstdlib>
#endif

namespace MUSIC
{

  Scheduler::Scheduler (MPI_Comm comm, int leader) :
      nodes (NULL), appl_data_ (NULL), conn_data_ (NULL)
  {
    comm_ = comm;
    leader_ = leader;
  }


  Scheduler::~Scheduler ()
  {
    if (nodes != NULL)
      delete nodes;
    if (appl_data_ != NULL)
      delete[] appl_data_;
    if (conn_data_ != NULL)
      delete[] conn_data_;
  }


  void
  Scheduler::initialize (TemporalNegotiatorGraph *appl_graph,
      std::vector<Connector*>& connectors)
  {

    nodes = new SGraph (appl_graph->local_node_color (), appl_graph->nNodes (),
        appl_graph->getNConnections ());

    setApplications (appl_graph);

    setConnections (appl_graph, connectors);

    color_ = appl_graph->local_node_color ();

    iter_node = nodes->begin ();
    iter_conn = 0;

    pushForward ();

    std::vector<SchedulerAgent *>::iterator agent;
    for (agent = agents_.begin(); agent != agents_.end(); agent++)
      (*agent)->initialize(connectors);

    initializeAgentState ();

  }


  void
  Scheduler::setAgent (SchedulerAgent* agent)
  {
    agents_.push_back (agent);
  }


  void
  Scheduler::initializeAgentState ()
  {
    cur_agent_ = agents_.begin ();
  }


  AEdge<Scheduler::SApplData, Scheduler::SConnData>
  Scheduler::pushForward ()
  {
    while (true)
      {
#if 0
        //	std::vector<Node*>::iterator node;
        for (; iter_node < nodes.size (); ++iter_node )
          {
            Node *node = nodes.at(iter_node);
            if (iter_conn < 0 && node->nextReceive () > node->localTime ().time ())
            node->advance ();

            std::vector<SConnection*>* conns = node->outputConnections ();
            // std::vector<SConnection*>::iterator conn;

            for (++iter_conn;
                iter_conn < static_cast<int> (conns->size ());
                ++iter_conn)
              {
                SConnection *conn = conns->at(iter_conn);
                //do we have data ready to be sent?
                if (conn->nextSend () <= conn->preNode ()->localTime ()
                    && conn->nextReceive () == conn->postNode ()->localTime ())
                  {
                    SConnection sconn = *conn;
                    sconn.postponeNextSend( conn->preNode()->localTime());
                    conn->advance();
                    return sconn;
                  }
              }
            iter_conn = -1;
          }
#endif
        for (; iter_node < nodes->end (); iter_node++)
          {
            if (iter_conn == 0)
              iter_conn = (*iter_node).begin_o ();
            else
              {
                advanceConnection ( (**iter_conn).data ());
                ++iter_conn;
              }
            for (; iter_conn < (*iter_node).end_o (); iter_conn++)
              {

                //do we have data ready to be sent?
                if ( (*iter_conn)->data ().nextSend
                    <= (*iter_conn)->pre ().data ().localTime
                    && (*iter_conn)->data ().nextReceive
                        == (*iter_conn)->post ().data ().localTime)
                  {
                    last_sconn_ = * (*iter_conn);

                    last_sconn_.data ().postponeNextSend (
                        (*iter_conn)->pre ().data ().localTime);

                    return last_sconn_;
                  }
              }

            iter_conn = 0;
          }


        for (iter_node = nodes->begin (); iter_node < nodes->end ();
            iter_node++)
          {
            Clock next_recv = nextApplicationReceive (*iter_node);
            if ((*iter_node).data ().localTime < next_recv)
              (*iter_node).data ().localTime.tick ();
          }
        iter_node = nodes->begin ();

      }
  }

#if 0
  void
  Scheduler::createMultiConnectors (Clock localTime,
      std::vector<Connector*>& connectors,
      MultiBuffer* multiBuffer,
      std::vector<MultiConnector*>& multiConnectors)
    {
      //if (mpi_get_rank (MPI_COMM_WORLD) == 2)
      //  std::cout << "prep localTime = " << localTime.time () << std::endl;
      // We need to create the MultiConnectors ahead of time since their
      // creation requires communication between all members of
      // COMM_WORLD.  Temporary solution: Create all MultiConnectors
      // that will be needed during the first 100 steps.
      std::vector<std::pair<double, Connector *> > schedule;

      std::vector<bool> multiProxies;
      multiProxies.resize (Connector::proxyIdRange ());

      for (int self_node = 0; self_node < (int) nodes.size (); ++self_node)
        {
          localTime.reset ();

          createMultiConnNext (self_node,
              localTime,
              connectors,
              multiBuffer,
              schedule);

          localTime.ticks (-1);

          std::vector<Connector*> cCache;
          for (int i = 0; i < 100; ++i)
            {
              localTime.tick ();
              createMultiConnStep (self_node,
                  localTime,
                  connectors,
                  multiBuffer,
                  multiConnectors,
                  multiProxies,
                  cCache,
                  schedule,
                  false);
            }
          createMultiConnStep (self_node,
              localTime,
              connectors,
              multiBuffer,
              multiConnectors,
              multiProxies,
              cCache,
              schedule,
              true);
          schedule.clear ();

          // Now reset node and connection clocks to starting values
          for (std::vector<Node*>::iterator node = nodes.begin ();
              node != nodes.end ();
              ++node)
          (*node).localTime.reset ();

          for (std::vector<SConnection*>::iterator conn = connections.begin ();
              conn != connections.end ();
              ++conn)
            {
              (*conn)->resetClocks ();
              (*conn)->advance ();
            }
        }
    }

  void
  Scheduler::createMultiConnNext (int self_node,
      Clock& localTime,
      std::vector<Connector*>& connectors,
      MultiBuffer* multiBuffer_,
      std::vector<std::pair<double, Connector *> > &schedule)
    {
      while (schedule.empty ()
          // always plan forward past the first time point to make
          // sure that all events at that time are scheduled
          || schedule.front ().first == schedule.back ().first)
        {
          std::vector<Node*>::iterator node;
          for ( node=nodes.begin (); node < nodes.end (); ++node )
            {
              if ((*node)->nextReceive () > (*node)->localTime ().time ())
              (*node)->advance ();

              std::vector<SConnection*>* conns = (*node)->outputConnections ();
              std::vector<SConnection*>::iterator conn;
              for (conn = conns->begin (); conn < conns->end (); ++conn)
              //do we have data ready to be sent?
              if ((*conn)->nextSend () <= (*conn)->preNode ()->localTime ()
                  && (*conn)->nextReceive () == (*conn)->postNode ()->localTime ())
                {
                  if (self_node == (*conn)->postNode ()->getId () //input
                      || self_node == (*conn)->preNode ()->getId ())//output
                    {
                      double nextComm
                      = (self_node == (*conn)->postNode ()->getId ()
                          ? (*conn)->postNode ()->localTime ().time ()
                          : (*conn)->preNode ()->localTime ().time ());
                      schedule.push_back
                      (std::pair<double, Connector*>(nextComm,
                              (*conn)->getConnector ()));
                    }

                  MUSIC_LOG0 ("Scheduled communication:"<< (*conn)->preNode ()->getId () <<"->"<< (*conn)->postNode ()->getId () << "at(" << (*conn)->preNode ()->localTime ().time () << ", "<< (*conn)->postNode ()->localTime ().time () <<")");
                  (*conn)->advance ();
                }
            }
        }
    }

  void
  Scheduler::createMultiConnStep (int self_node,
      Clock& localTime,
      std::vector<Connector*>& connectors,
      MultiBuffer* multiBuffer_,
      std::vector<MultiConnector*>& multiConnectors,
      std::vector<bool>& multiProxies,
      std::vector<Connector*>& cCache,
      std::vector<std::pair<double, Connector *> > &schedule,
      bool finalize)
    {
      std::set<int>* connsToFinalize;
      int finalCount = 0;

      if (finalize)
        {
          connsToFinalize = new std::set<int>;
          for (std::vector<Connector*>::iterator c = connectors.begin ();
              c != connectors.end ();
              ++c)
          connsToFinalize->insert ((*c)->receiverPortCode ());
        }

      while (schedule[0].first <= localTime.time()
          || (finalize && //!connsToFinalize->empty ()
              ++finalCount <= 4000))
        {
          std::vector< std::pair<double, Connector*> >::iterator comm;
          for (comm = schedule.begin ();
              comm != schedule.end() && comm->first <= localTime.time ();
              ++comm)
            {
              Connector* connector = comm->second;

              if (connector == NULL)
              continue;

              if (finalize)
              connsToFinalize->erase (connector->receiverPortCode ());

              if (!connector->idFlag ())
              continue;

              unsigned int multiId = 0;
              unsigned int multiProxyId = 0;
              if (!connector->isProxy ())
                {
                  multiId = connector->idFlag ();
                  cCache.push_back (connector);
                }
              else
                {
                  dynamic_cast<ProxyConnector*> (connector)->setNode (self_node);
                  multiProxyId = connector->idFlag ();
                }
              bool isProxy = connector->isProxy ();
              int remoteLeader = connector->remoteLeader ();
              bool direction = connector->isInput ();
              // yes, horrible quadratic algorithm for now
              for (std::vector< std::pair<double, Connector*> >::iterator c
                  = comm + 1;
                  c != schedule.end () && c->first <= localTime.time();
                  ++c)
                {
                  if (c->second == NULL || c->second->idFlag () == 0)
                  continue;
                  if (c->second->isProxy ())
                  dynamic_cast<ProxyConnector*> (c->second)->setNode (self_node);
                  // lumping criterion
                  if (!(c->second->isProxy () ^ isProxy)
                      && c->second->remoteLeader () == remoteLeader
                      && c->second->isInput () == direction)
                    {
                      if (finalize)
                      connsToFinalize->erase (c->second->receiverPortCode ());
                      if (!c->second->isProxy ())
                        {
                          cCache.push_back (c->second);
                          multiId |= c->second->idFlag ();
                        }
                      else
                        {
                          multiProxyId |= c->second->idFlag ();
                        }
                      c->second = NULL; // taken
                    }
                }

              //if mpi_get_rank ((MPI_COMM_WORLD) == 2)
              //	std::cout << "Prep multiId = " << multiId << std::endl;
              if (cCache.size () == 0)
                {
                  if (!multiProxies[multiProxyId])
                    {
                      std::cout << "Rank " << mpi_get_rank (MPI_COMM_WORLD)
				<< ": *" << std::endl << std::flush;
		      MPI_Comm comm;
                      MPI_Comm_create (MPI_COMM_WORLD, MPI_GROUP_EMPTY, &comm);
                      MPI_Barrier (MPI_COMM_WORLD);
                      //*fixme* Impossible to delete the following object
                      multiProxies[multiProxyId] = true;
                    }
                }
              else if (multiConnectors[multiId] == NULL)
                {
                  multiConnectors[multiId]
                  = new MultiConnector (multiBuffer_, cCache);
                }
              cCache.clear ();
            }
          schedule.erase (schedule.begin (), comm);
          createMultiConnNext (self_node,
              localTime,
              connectors,
              multiBuffer_,
              schedule);
        }

      if (finalize)
      delete connsToFinalize;

    }
#endif


  void
  Scheduler::reset (int color)
  {
    if (color < 0)
      color = nodes->local_node_color ();
    color_ = color;
    reset ();
    nodes->setTempColor (color);
    std::vector<SConnection> path;
    nodes->depthFirst (nodes->at (color), path);

    cur_agent_ = agents_.begin ();
    iter_node = nodes->begin ();
    iter_conn = 0;
    pushForward ();

  }


  void
  Scheduler::reset ()
  {
    nodes->reset ();
    for (SGraph::iterator n = nodes->begin (); n < nodes->end (); n++)
      {
        (*n).data ().localTime.reset ();
        for (SNode::edge_iterator e = (*n).begin_o (); e < (*n).end_o (); e++)
          {
            (*e)->data ().reset ();
            advanceConnection ( (**e).data ());
          }
      }
  }


  void
  Scheduler::tick (Clock& localTime)
  {
    // tick is done when at least one of the agent will schedule the future communication
    // next tick will start from the last successful agent
    // this algorithm is based on temporally ordered SConnections
    bool done = false;

    for (; !done; cur_agent_++)
      {
        if (cur_agent_ == agents_.end ())
          cur_agent_ = agents_.begin ();

        if ( (*cur_agent_)->tick (localTime))
          done = true;
      }
  }


  void
  Scheduler::finalize (Clock& localTime, std::vector<Connector*>& connectors)
  {
    /* remedius
     * set of receiver port codes that still has to be finalized
     */

    std::set<int> cnn_ports;
    for (std::vector<Connector*>::iterator c = connectors.begin ();
        c != connectors.end (); ++c)
#if 0
      if (! (*c)->needsMultiCommunication ())
#endif
        {
          cnn_ports.insert ( (*c)->receiverPortCode ());
        }
#if 0
    for (std::vector<SchedulerAgent*>::iterator a = agents_.begin ();
        a != agents_.end (); ++a)
      (*a)->preFinalize (cnn_ports);
#endif

    for (; !cnn_ports.empty (); cur_agent_++)
      {
        if (cur_agent_ == agents_.end ())
          cur_agent_ = agents_.begin ();
        (*cur_agent_)->finalize (cnn_ports);
      }
  }


  Clock
  Scheduler::nextApplicationReceive (SNode &node)
  {
    SNode::edge_iterator in_conn;
    Clock nextTime;
    nextTime.set(ClockState(LLONG_MAX));
    for (in_conn = node.begin_i (); in_conn < node.end_i (); ++in_conn)
      {
        SConnData &data = (*in_conn)->data ();
        if (data.nextReceive < nextTime)
          nextTime = data.nextReceive;
      }
    return nextTime;
  }


  void
  Scheduler::advanceConnection (SConnData &data)
  {
    advanceConnectionClocks (data);
    Clock r = data.nextReceive;
    Clock s = data.nextSend;
    advanceConnectionClocks (data);
    while (data.nextReceive == r)
      {
        s = data.nextSend;
        advanceConnectionClocks (data);
      }
    data.nextReceive = r;
    data.nextSend = s;
    MUSIC_LOG0 (pre_id << "->"<<post_id << "::"<<nextSend_.time () << "::" << nextReceive_.time ());
  }


  void
  Scheduler::advanceConnectionClocks (SConnData &data)
  {
    ClockState limit = (data.nextSend.integerTime () + data.descr.accLatency
        - data.nextReceive.tickInterval ());
    while (data.nextReceive.integerTime () <= limit)
      data.nextReceive.tick ();
    data.nextSend.ticks (data.descr.maxBuffered + 1);
  }


  void
  Scheduler::setApplications (TemporalNegotiatorGraph *appl_graph)
  {
    appl_data_ = new SApplData[appl_graph->nNodes ()];
    int i = 0;
    TemporalNegotiatorGraph::iterator it;
    for (it = appl_graph->begin (); it < appl_graph->end (); ++it, ++i)
      {
        appl_data_[i].color = (*it).data ().color;
        appl_data_[i].leader = (*it).data ().leader;
        appl_data_[i].nProcs = (*it).data ().nProcs;
        appl_data_[i].localTime.configure ( (*it).data ().timebase,
            (*it).data ().tickInterval);
        nodes->addNode (
            SNode ( (*it).data ().nOutConnections, (*it).data ().nInConnections,
                appl_data_[i]), (*it).data ().color);
      }

  }


  void
  Scheduler::setConnections (TemporalNegotiatorGraph *appl_graph,
      std::vector<Connector*>& connectors)
  {
    conn_data_ = new SConnData[appl_graph->getNConnections ()];
    int k = 0;
    SGraph::iterator inode;
    for (inode = nodes->begin (); inode < nodes->end (); inode++)
      {
        TemporalNegotiationData &data =
            appl_graph->at ( (*inode).data ().color).data ();
        for (int i = 0; i < data.nOutConnections; ++i)
          {
            Connector *connector;
            bool foundLocalConnector = false;
            ConnectionDescriptor &descr = data.connection[i];
            SNode &pre = *inode;
            SNode &post = nodes->at (descr.remoteNode);

            for (std::vector<Connector*>::iterator c = connectors.begin ();
                c != connectors.end (); ++c)
              {
                if (descr.receiverPort == (*c)->receiverPortCode ())
                  {
                    foundLocalConnector = true;
                    connector = *c;
                    (*c)->setInterpolate (descr.interpolate);
                    (*c)->setLatency (descr.accLatency);
                    (*c)->initialize ();
                    break;
                  }
              }
#if 1
            if (!foundLocalConnector)
              {
                if (descr.multiComm)
                  connector = new ProxyConnector (pre.data ().color,
                      pre.data ().leader, pre.data ().nProcs,
                      post.data ().color, post.data ().leader,
                      post.data ().nProcs);
                else
                  connector = new ProxyConnector ();
              }

#endif
            conn_data_[k].descr = descr;
            conn_data_[k].connector = connector;
            conn_data_[k].nextSend.configure (pre.data ().localTime.timebase (),
                pre.data ().localTime.tickInterval ());
            conn_data_[k].nextReceive.configure (
                post.data ().localTime.timebase (),
                post.data ().localTime.tickInterval ());
            SConnection &edge = nodes->addEdge (
                SConnection (pre, post, conn_data_[k++]));

            advanceConnection (edge.data ());

          }
      }

  }


  void
  Scheduler::SGraph::handleLoop (SNode &x, std::vector<SConnection> & path)
  {
    if (&x == &nodes_[tmp_color_])
      {
        // Mark connections on path as loop connected to self_node
        for (std::vector<SConnection>::iterator c = path.begin ();
            c != path.end (); ++c)
          (*c).data ().isLoopConnected = true;
      }
  }

}

#endif
