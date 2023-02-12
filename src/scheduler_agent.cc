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

#include <music/scheduler_agent.hh>

//#define MUSIC_DEBUG
#include "music/debug.hh"

#if MUSIC_USE_MPI

#include <algorithm>
#include <iostream>
#include <cassert>
#ifdef MUSIC_DEBUG
#include <cstdlib>
#endif

namespace MUSIC
{
  SchedulerAgent::SchedulerAgent (Scheduler *scheduler) :
      scheduler_ (scheduler)
  {

  }


  MulticommAgent::MulticommAgent (Scheduler *scheduler) :
      SchedulerAgent (scheduler)
  {
    multiProxies = NULL;
    multiBuffer_ = NULL;
    filter1 = NULL;
    filter2 = NULL;
  }


  MulticommAgent::Filter1::Filter1 (MulticommAgent &multCommObj) :
      multCommObj_ (multCommObj)
  {

  }


  /*    remedius
   *  This filter is applied to the set of selected connectors in the fillSchedule().
   *  It partitioned the set of connectors into two groups:
   *  returns True: connectors that can be lumped:if scheduled send times <= multCommObj_.time_.
   *  This condition is sufficient because each conflicting send time can be postponed to the most latter scheduled for the same node.
   *  The most latter possible time is a multCommObj_.time_ (the receive time).
   *  returns False: connectors that go to the Filter2 (connectors that possibly can be added to the current group)
   */
  bool
  MulticommAgent::Filter1::operator() (SConnectionP &conn)
  {
    if (conn.second.sSend <= multCommObj_.time_)
      {
        int sId = conn.first.pre ().data ().color;
        int rId = conn.first.post ().data ().color;
        //keeps track of all receive nodes
        multCommObj_.rNodes |= 1 << rId;
        //guarantees that if there are both send and receive are scheduled to the same node,
        // the receive time will be chosen.
        multCommObj_.commTimes[rId] = multCommObj_.time_;
        // take the most later time
        if (conn.second.sSend >= multCommObj_.commTimes[sId])
          multCommObj_.commTimes[sId] = conn.second.sSend;
        return true;
      }
    return false;
  }


  MulticommAgent::Filter2::Filter2 (MulticommAgent &multCommObj) :
      multCommObj_ (multCommObj)
  {

  }


  /*    remedius
   * This filter is applied to the rest of the selected connectors, that were classified as False by the Filter1.
   * It adds those of the rest connectors(which scheduled send times > multCommObj_.time_.) that either don't have a conflict
   * with the already selected connectors by Filter1 or if the conflict is solvable.
   * The conflict is not solvable if both send and receive are scheduled for the same node.
   */
  bool
  MulticommAgent::Filter2::operator() (SConnectionP &conn)
  {
    int sId = conn.first.pre ().data ().color;
    int rId = conn.first.post ().data ().color;
    std::map<int, Clock>::iterator it;
    it = multCommObj_.commTimes.find (sId);
    if (it == multCommObj_.commTimes.end ()
        || ! (multCommObj_.rNodes & (1 << sId)))
      {
        multCommObj_.commTimes[sId] = conn.second.sSend;
        multCommObj_.commTimes[rId] = multCommObj_.time_;
        multCommObj_.rNodes |= 1 << rId;
        return true;
      }
    return false;
  }


  MulticommAgent::~MulticommAgent ()
  {
    for (std::vector<MultiConnector*>::iterator connector =
        multiConnectors.begin (); connector != multiConnectors.end ();
        ++connector)
      if (*connector != NULL)
        delete *connector;
    if (multiProxies != NULL)
      delete multiProxies;
    if (multiBuffer_ != NULL)
      delete multiBuffer_;
    if (filter1 != NULL)
      delete filter1;
    if (filter2 != NULL)
      delete filter2;
  }



  void
  MulticommAgent::initialize (std::vector<Connector*>& connectors)
  {
    filter1 = new Filter1 (*this);
    filter2 = new Filter2 (*this);
    multiBuffer_ = new MultiBuffer (scheduler_->comm(), scheduler_->getLeader(), connectors);
    //contains the multiconnectors which were created (I'm included)
    multiConnectors.resize (Connector::idRange ());
    //denotes multiIds which were created (I'm not included)
    multiProxies = new std::vector<bool>;
    multiProxies->resize (Connector::proxyIdRange ());
    for (unsigned int i = 0; i < Connector::idRange (); ++i)
      multiConnectors[i] = NULL;
    for (unsigned int i = 0; i < Connector::proxyIdRange (); ++i)
      (*multiProxies)[i] = false;

  }


#if 0
  void
  MulticommAgent::createMultiConnectors (Clock& localTime, MPI_Comm comm,
      int leader, std::vector<Connector*>& connectors)
  {
    multiBuffer_ = new MultiBuffer (comm, leader, connectors);
    multiConnectors.resize (Connector::idRange ());
    multiProxies = new std::vector<bool>;
    multiProxies->resize (Connector::proxyIdRange ());

    for (unsigned int self_node = 0; self_node < scheduler_->nApplications ();
        ++self_node)
      {

        scheduler_->reset (self_node);

        if (!create (localTime))
          scheduler_->pushForward ();

        localTime.ticks (-1);

        for (int i = 0; i < N_PLANNING_CYCLES; ++i)
          {
            localTime.tick ();
            if (!create (localTime))
              scheduler_->pushForward ();

          }

        //finalize
        schedule.clear ();

        localTime.reset ();
      }

    scheduler_->reset (-1);
    delete multiProxies;
  }

#endif

  std::vector<Connector*>
  MulticommAgent::connectorsFromMultiId (unsigned int multiId)
  {
    std::vector<Connector*> connectors;
    for (unsigned int flag = 1; multiId != 0; flag <<= 1)
      if (multiId & flag)
        {
          connectors.push_back (Connector::connectorFromIdFlag (flag));
          multiId &= ~flag;
        }
    return connectors;
  }

#if 0
  bool
  MulticommAgent::create (MUSIC::Clock &localTime)
  {
    std::vector<MultiCommObject>::iterator comm;
    bool continue_;
    do
      {
        continue_ = fillSchedule ();
        for (comm = schedule.begin ();
            comm != schedule.end () && (*comm).time <= localTime;
            ++comm)
          {
            unsigned int multiId = (*comm).multiId ();
            if (multiId != 0)
              {
                if (multiConnectors[multiId] == NULL)
                  {
                    std::vector<Connector*> connectors = connectorsFromMultiId (
                        multiId);
                    multiConnectors[multiId] = new MultiConnector (multiBuffer_,
                        connectors);
                  }
              }
            else
              {
                unsigned int proxyId = (*comm).proxyId ();
                if (proxyId != 0 && ! (*multiProxies)[proxyId])
                  {
                    //  std::cout << "Rank " << mpi_get_rank (MPI_COMM_WORLD)
                    //      << ": Proxy " << proxyId << std::endl;
		    MPI_Comm newcomm;
                    MPI_Comm_create (MPI_COMM_WORLD, MPI_GROUP_EMPTY, &newcomm);
                    MPI_Barrier (MPI_COMM_WORLD);
                    (*multiProxies)[proxyId] = true;
                  }
              }
          }
        schedule.erase (schedule.begin (), comm);
      }
    while (continue_ && schedule.empty ());
    return !schedule.empty ();
  }

#endif

  bool
  MulticommAgent::tick (MUSIC::Clock &localTime)
  {
    std::vector<MultiCommObject>::iterator comm;
    bool continue_;
    do
      {
        continue_ = fillSchedule ();
        for (comm = schedule.begin ();
            comm != schedule.end () && (*comm).time <= localTime; ++comm)
          {
            unsigned int multiId = (*comm).multiId ();
            unsigned int proxyId = (*comm).proxyId ();
            // if we participate in the multicommunication but the multiconnector was not yet created
            if (multiId != 0 && multiConnectors[multiId] == NULL)
              {
                std::vector<Connector*> connectors = connectorsFromMultiId (
                    multiId);
                multiConnectors[multiId] = new MultiConnector (multiBuffer_,
                    connectors);

              }// if we do not participate in the multicommunication and the multiconnector was not yet created
            else if (multiId == 0 && !((*multiProxies)[proxyId]))
              {
		MPI_Comm newcomm;
                MPI_Comm_create (MPI_COMM_WORLD, MPI_GROUP_EMPTY, &newcomm);
                MPI_Barrier (MPI_COMM_WORLD);
                (*multiProxies)[proxyId] = true;
              }

            if ( multiId != 0)
              {
                assert (multiConnectors[multiId] != NULL);
                multiConnectors[multiId]->tick ();
              }
          }
        schedule.erase (schedule.begin (), comm);
        // we continue looping while:
        // 1. we haven't found the communication from the future:   !schedule.empty()
        // 2. or the next SConnection can't be processed by the current agent: continue_= false
      }
    while (continue_ && schedule.empty ());
    return !schedule.empty ();
  }


  // *schedule object can be either non empty or empty
  // if it's empty it will be filled with collective connectors if appropriate
  // if it's non empty, it contains collective connectors filled in before
  // *last_sconn can be either p2p or collective connector
  bool
  MulticommAgent::fillSchedule ()
  {
    if (!schedule.empty ())
      return true;
    Scheduler::SConnection last_sconn = scheduler_->getLastSConnection ();
    if (!last_sconn.data ().connector->idFlag ())
      return false;
    Clock nextReceive = last_sconn.data ().nextReceive; //last connection
    SConnectionPV nextSConnections;
    do
      {
#if 0
        if (last_sconn.data ().isLoopConnected
            || scheduler_->isLocalConnection (last_sconn))
#endif
          nextSConnections.push_back (
              std::make_pair (last_sconn, last_sconn.data ()));

        last_sconn = scheduler_->pushForward ();
      }
    while (last_sconn.data ().connector->idFlag () //those that request multicommunication
    && (last_sconn.data ().nextReceive == nextReceive)); //those that have scheduled receive at current time
    //  || (!last_sconn.data ().isLoopConnected //
    //  && !scheduler_->isLocalConnection (last_sconn))));

    if (nextSConnections.size () > 0)
      NextMultiConnection (nextSConnections);

    return last_sconn.data ().connector->idFlag ();
  }


  void
  MulticommAgent::NextMultiConnection (SConnectionPV &candidates)
  {
    time_ = candidates[0].second.nextReceive;
    // std::map<int, Clock> prevCommTime;
    SConnectionPV::iterator iter_bound = candidates.begin ();
    SConnectionPV::iterator cur_bound = candidates.begin ();
    do
      {
        cur_bound = iter_bound;

        commTimes.clear ();
        rNodes = 0;

        // after the first iteration inner partitioning (Filter1) will always return .begin()
        iter_bound = std::stable_partition (
            std::stable_partition (cur_bound, candidates.end (), *filter1),
            candidates.end (), *filter2);

        //postpone sends in the multiconn for consistency
        for (SConnectionPV::iterator it = cur_bound; it != iter_bound; ++it)
          (*it).second.postponeNextSend (
              commTimes[ (*it).first.pre ().data ().color]);

        std::map<int, Clock>::iterator id_iter;
        if ( (id_iter = commTimes.find (scheduler_->self_node ()))
            != commTimes.end ())
          scheduleMulticonn ( (*id_iter).second, cur_bound, iter_bound);
      }
    while (iter_bound != candidates.end ());
  }


  void
  MulticommAgent::scheduleMulticonn (Clock &time, SConnectionPV::iterator first,
      SConnectionPV::iterator last)
  {
    unsigned int multiId = 0;
    unsigned int proxyId = 0;
    MUSIC_LOGR ("SelfNode: " << scheduler_->self_node() << " Time: "<< time.time() << std::endl << "MultiConnector:");
    for (SConnectionPV::iterator it = first; it != last;
        ++it)
      {
        if ( (*it).second.connector->isProxy ())
          proxyId |= (*it).second.connector->idFlag ();
        else
          multiId |= (*it).second.connector->idFlag ();
        MUSIC_LOGR ("("<< (*it).preNode ()->getId () <<" -> "<< (*it).postNode ()->getId () << ") at " << (*it).nextSend().time () << " -> "<< (*it).nextReceive ().time ());
      }
    schedule.push_back (MultiCommObject (time,  multiId , proxyId));
  }

#if 0
  void
  MulticommAgent::preFinalize (std::set<int> &cnn_ports)
  {
    for (std::vector<MultiConnector*>::iterator m = multiConnectors.begin ();
        m != multiConnectors.end (); ++m)
      if (*m != NULL)
        cnn_ports.insert ( (*m)->connectorCode ());
  }
#endif

  void
  MulticommAgent::finalize (std::set<int> &cnn_ports)
  {
    std::vector<MultiCommObject>::iterator comm;
    bool continue_;
    do
      {
        continue_ = fillSchedule ();
        for (comm = schedule.begin ();
            comm != schedule.end () && !cnn_ports.empty (); ++comm)
          {
            unsigned int multiId = (*comm).multiId ();
            unsigned int proxyId = (*comm).proxyId ();
            // if we participate in the multicommunication but the multiconnector was not yet created
            if (multiId != 0 && multiConnectors[multiId] == NULL)
              {
                std::vector<Connector*> connectors = connectorsFromMultiId (
                    multiId);
                multiConnectors[multiId] = new MultiConnector (multiBuffer_,
                    connectors);
              } // if we do not participate in the multicommunication and the multiconnector was not yet created
            else if (multiId == 0 && ! ( (*multiProxies)[proxyId]))
              {
		MPI_Comm newcomm;
                MPI_Comm_create (MPI_COMM_WORLD, MPI_GROUP_EMPTY, &newcomm);
                MPI_Barrier (MPI_COMM_WORLD);
                (*multiProxies)[proxyId] = true;

              }
            if (multiId != 0)
              {
                MultiConnector* m = multiConnectors[multiId];
#if 0
                if (cnn_ports.find (m->connectorCode ()) == cnn_ports.end ())
                  continue;
#endif
                if (m->isFinalized ())
                // an output port was finalized in tick ()
                  {
#if 0
                    cnn_ports.erase (m->connectorCode ());
#endif
                    std::vector<Connector*> conns = connectorsFromMultiId (multiId);
                    std::vector<Connector *>::iterator it;
                    for(it = conns.begin(); it != conns.end(); ++it)
                      cnn_ports.erase ((*it)->receiverPortCode());
                    continue;
#if 0
                    // Sometimes the scheduler fails to generate all
                    // multiConnectors during finalization.  Here, we
                    // weed out multiConnectors that were finalized as
                    // a consequence of the finalization of m.
                    for (std::vector<MultiConnector*>::iterator m =
                        multiConnectors.begin (); m != multiConnectors.end ();
                        ++m)
                      if (*m != NULL
                          && (cnn_ports.find ( (*m)->connectorCode ())
                              != cnn_ports.end ()) && (*m)->isFinalized ())
                        cnn_ports.erase ( (*m)->connectorCode ());
                    continue;
#endif
                  }
                // finalize () needs to come after isFinalized check
                // since it can itself set finalized state
                m->finalize ();

                m->tick ();
              }
          }
        schedule.clear ();
      }
    while (continue_ && !cnn_ports.empty ());
  }


  UnicommAgent::UnicommAgent (Scheduler *scheduler) :
      SchedulerAgent (scheduler)
  {
  }


  // *schedule object can be either non empty or empty
  // if it's empty it will be filled with p2p connector if appropriate
  // if it's non empty, it contains p2p connectors filled in before
  // *last_sconn can be either p2p or collective connector
  bool
  UnicommAgent::fillSchedule ()
  {
    if (!schedule.empty ())
      return true;
    Scheduler::SConnection last_sconn = scheduler_->getLastSConnection ();
    if (last_sconn.data ().connector->idFlag ())
      return false;
    if (scheduler_->isLocalConnection (last_sconn))
      {
        Clock nextComm = (
            scheduler_->self_node () == last_sconn.post ().data ().color ?
                last_sconn.data ().nextReceive :
                last_sconn.data ().nextSend);
        schedule.time = nextComm;
        schedule.connector = last_sconn.data ().connector;
        MUSIC_LOGR (" :Scheduled communication:"<< last_sconn.preNode ()->getId () <<"->"
            << last_sconn.postNode ()->getId ()<< "at("
            << last_sconn.nextSend ().time () << ", "
            << last_sconn.nextReceive ().time () <<")");

      }
    scheduler_->pushForward ();
    return true;
  }


  bool
  UnicommAgent::tick (Clock& localTime)
  {
    bool continue_;
    do
      {
        continue_ = fillSchedule ();
        if (!schedule.empty () && schedule.time <= localTime)
          {
            schedule.connector->tick ();
            schedule.reset ();
          }
      }
    while (continue_ && schedule.empty ());
    return !schedule.empty ();
  }


  void
  UnicommAgent::finalize (std::set<int> &cnn_ports)
  {
    bool continue_;
    do
      {
        continue_ = fillSchedule ();
        if (!schedule.empty ())
          {
            Connector* connector = schedule.connector;
            schedule.reset ();
            if ( (cnn_ports.find (connector->receiverPortCode ())
                == cnn_ports.end ()))
              continue;
            if (connector->isFinalized ())
            // an output port was finalized in tick ()
              {
                cnn_ports.erase (connector->receiverPortCode ());
                continue;
              }
            // finalize () needs to come after isFinalized check
            // since it can itself set finalized state
            connector->finalize ();
          }
      }
    while (continue_ && !cnn_ports.empty ());
  }

}
#endif
