/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007-2012, 2022 INCF
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
//#define MUSIC_AFTER_RUNTIME_CONSTRUCTOR_REPORT
#include "music/runtime.hh"

#if MUSIC_USE_MPI

#include <mpi.h>

#include "music/temporal.hh"
#include "music/error.hh"
#include "music/connection.hh"
#include "music/memory.hh"

#include <algorithm>
#include <iostream>
#include <set>
#include <cassert>

namespace MUSIC
{

  bool Runtime::isInstantiated_ = false;


  Runtime::Runtime (Setup* s, double h)
    : mAgent (0)
  {
    checkInstantiatedOnce (isInstantiated_, "Runtime");
    s->maybePostponedSetup ();

    app_name = s->applicationName ();
    leader_ = s->leader ();

    /* remedius
     * new type of subconnectors for collective communication was created.
     */
    //CollectiveSubconnectors collectiveSubconnectors;
    // Setup the MUSIC clock
    localTime = Clock (s->timebase (), h);

    comm = s->communicator ();

    /* remedius
     * copy ports in order to delete them afterwards (it was probably a memory leak in previous revision)
     */
    for (std::vector<Port *>::iterator it = s->ports ()->begin ();
        it < s->ports ()->end (); it++)
      ports.push_back ( (*it));

    Connections* connections = s->connections ();

    scheduler = new Scheduler (comm, leader_);
    if (s->launchedByMusic ())
      {
        takeTickingPorts (s);

        // create a total order for connectors and
        // establish connection to peers
        connectToPeers (connections);

        // specialize connectors and fill up connectors vector
        specializeConnectors (connections);
        // from here we can start using the vector `connectors'
        // negotiate where to route data and fill up subconnector vectors
        spatialNegotiation ();

        // build data routing tables
        buildTables (s);
        takePreCommunicators ();
        takePostCommunicators ();
        // negotiate timing constraints for synchronizers
        temporalNegotiation (s, connections);
#if 0
        if (needsMultiCommunication ())
          {
#endif
        sAgents.push_back (mAgent = new MulticommAgent (scheduler));
        scheduler->setAgent (mAgent);
#if 0
      }
#endif
        sAgents.push_back (new UnicommAgent (scheduler));
        scheduler->setAgent (sAgents[sAgents.size () - 1]);

        // final initialization before simulation starts
        initialize (s);
      }
    else
      {
	sAgents.push_back (new DummyAgent (scheduler));
	scheduler->setAgent (sAgents[0]);
	scheduler->initializeAgentState ();
      }

    delete s;
#ifdef MUSIC_AFTER_RUNTIME_CONSTRUCTOR_REPORT
    if (mpi_get_rank (MPI_COMM_WORLD) == 0)
    reportMem ();
#if 0
    // Code used for debugging and analysis
    for (std::vector<Connector*>::iterator connector = connectors.begin ();
        connector != connectors.end ();
        ++connector)
    (*connector)->report ();
#endif
#endif
  }


  Runtime::~Runtime ()
  {
    // delete connectors
    for (std::vector<Connector*>::iterator connector = connectors.begin ();
        connector != connectors.end (); ++connector)
      delete *connector;
#if 0
    /* MDJ 2012-08-07
     Some applications assume that they need to delete the ports.
     Until we have made a decision what is the proper usage pattern,
     this code is disabled. */

    // delete ports
    for (std::vector<Port *>::iterator it=ports.begin(); it < ports.end(); it++ )
    delete (*it);
#endif
    for (std::vector<SchedulerAgent *>::iterator it = sAgents.begin ();
        it != sAgents.end (); it++)
      delete (*it);
    delete scheduler;
    isInstantiated_ = false;
  }


  bool
  Runtime::needsMultiCommunication ()
  {
    std::vector<Connector*>::iterator c;
    for (c = connectors.begin (); c != connectors.end (); ++c)
      if ((*c)->needsMultiCommunication ())
	return true;
    return false;
  }

  void
  Runtime::takeTickingPorts (Setup* s)
  {
    std::vector<Port*>::iterator p;
    for (p = ports.begin (); p != ports.end (); ++p)
      {
        TickingPort* tp = dynamic_cast<TickingPort*> (*p);
        if (tp != NULL)
          tickingPorts.push_back (tp);
      }
  }


  void
  Runtime::takePreCommunicators ()
  {
    std::vector<Connector*>::iterator c;
    for (c = connectors.begin (); c != connectors.end (); ++c)
      {
        PreCommunicationConnector* preCommunicationConnector =
            dynamic_cast<PreCommunicationConnector*> (*c);
        if (preCommunicationConnector != NULL)
          preCommunication.push_back (preCommunicationConnector);
      }
  }


  void
  Runtime::takePostCommunicators ()
  {
    std::vector<Connector*>::iterator c;
    for (c = connectors.begin (); c != connectors.end (); ++c)
      {
        PostCommunicationConnector* postCommunicationConnector =
            dynamic_cast<PostCommunicationConnector*> (*c);
        if (postCommunicationConnector != NULL)
          postCommunication.push_back (postCommunicationConnector);
      }
  }

  // This predicate gives a total order for connectors which is the
  // same on the sender and receiver sides.  It belongs here rather
  // than in connector.hh or connector.cc since it is connected to the
  // connect algorithm.
  bool
  lessConnection (const Connection* cn1, const Connection* cn2)
  {
    Connector* c1 = cn1->connector ();
    Connector* c2 = cn2->connector ();
    return (c1->receiverAppName () < c2->receiverAppName ()
        || (c1->receiverAppName () == c2->receiverAppName ()
            && c1->receiverPortName () < c2->receiverPortName ()));
  }


  void
  Runtime::connectToPeers (Connections* connections)
  {
    // This ordering is necessary so that both sender and receiver
    // in each pair sets up communication at the same point in time
    //
    // Note that we don't need to use the dead-lock scheme used in
    // build_schedule () here.
    //
    sort (connections->begin (), connections->end (), lessConnection);

    for (Connections::iterator c = connections->begin ();
        c != connections->end (); ++c)
      (*c)->connector ()->createIntercomm ();
  }


  void
  Runtime::specializeConnectors (Connections* connections)
  {
    for (Connections::iterator c = connections->begin ();
        c != connections->end (); ++c)
      {
        Connector *connector = (*c)->connector ();
        connector->specialize (localTime);
        //(*c)->setConnector (connector);
        connectors.push_back (connector);
      }
  }


  void
  Runtime::spatialNegotiation ()
  {
    // Let each connector pair setup their inter-communicators
    // and create all required subconnectors.
    unsigned int multiId = 0;
    for (std::vector<Connector*>::iterator c = connectors.begin ();
        c != connectors.end (); ++c)
      {
        // negotiate and fill up vectors passed as arguments
        (*c)->spatialNegotiation ();
        multiId |= (*c)->idFlag ();
      }
  }


  void
  Runtime::buildTables (Setup* s)
  {
    for (std::vector<Port*>::iterator p = ports.begin (); p != ports.end ();
        ++p)
      (*p)->buildTable ();
  }


  void
  Runtime::temporalNegotiation (Setup* s, Connections* connections)
  {
    // Temporal negotiation is done globally by a serial algorithm
    // which yields the same result in each process
    s->temporalNegotiator ()->negotiate (localTime, connections);
  }


  MPI_Comm
  Runtime::communicator ()
  {
    return comm;
  }


  void
  Runtime::initialize (Setup* s)
  {
    scheduler->initialize (s->temporalNegotiator ()->applicationGraph (),
        connectors);
#if 0
    if (mAgent)
      mAgent->initialize (localTime, comm, leader_, connectors);
#endif
    // scheduler->nextCommunication (localTime, schedule);
    scheduler->tick (localTime);
    // compensate for first localTime.tick () in Runtime::tick ()
    localTime.ticks (-1);
    // the time zero tick () (where we may or may not communicate)
    tick ();
  }


  void
  Runtime::finalize ()
  {
#if 1
    scheduler->finalize (localTime, connectors);
#else
    /* remedius
     * set of receiver port codes that still has to be finalized
     */
    std::set<int> cnn_ports;
    for (std::vector<Connector*>::iterator c = connectors.begin ();
        c != connectors.end ();
        ++c)
    cnn_ports.insert ((*c)->receiverPortCode ());

    /*
     * finalize communication
     *
     * For ordinary connectors, Connector::finalize () is called
     * repeatedly until Connector::isFinalized () returns true.
     *
     * For connectors participating in multiConnectors,
     * Connector::finalize () is called, followed by a multi-tick
     * until Connector::isFInalized () returns true.
     */

    std::vector<Connector*> cCache;

    if (!schedule.empty ())
    do
      {
        std::vector<std::pair<double, Connector*> >::iterator comm;
        for (comm = schedule.begin (); comm != schedule.end (); ++comm)
          {
            Connector* connector = comm->second;
            if (connector == NULL
                || (cnn_ports.find (connector->receiverPortCode ())
                    == cnn_ports.end ()))
            continue; // already finalized

            if (!connector->idFlag ())
              {
                if (connector->isFinalized ())
                // an output port was finalized in tick ()
                  {
                    cnn_ports.erase (connector->receiverPortCode ());
                    continue;
                  }
                // finalize () needs to come after isFinalized check
                // since it can itself set finalized state
                connector->finalize ();
                continue;
              }

            // all multiconnector code should go into scheduler
            // this is a temporary solution
            int remoteLeader = connector->remoteLeader ();
            bool direction = connector->isInput ();
            unsigned int multiId = connector->idFlag ();
            cCache.clear ();
            cCache.push_back (connector);
            // yes, horrible quadratic algorithm for now
            for (std::vector< std::pair<double, Connector*> >::iterator c
                = comm + 1;
                c != schedule.end ();
                ++c)
              {
                if (c->second != NULL
                    && c->second->idFlag ()
                    && c->second->remoteLeader () == remoteLeader
                    && c->second->isInput () == direction)
                  {
                    cCache.push_back (c->second);
                    multiId |= c->second->idFlag ();
                    c->second = NULL; // taken
                  }
              }

            bool isFinalized = true;
            for (std::vector<Connector*>::iterator c = cCache.begin ();
                c != cCache.end ();
                ++c)
              {
                if (!(*c)->isFinalized ())
                isFinalized = false;
              }

            if (isFinalized)
              {
                for (std::vector<Connector*>::iterator c = cCache.begin ();
                    c != cCache.end ();
                    ++c)
                cnn_ports.erase ((*c)->receiverPortCode ());
                continue;
              }

            for (std::vector<Connector*>::iterator c = cCache.begin ();
                c != cCache.end ();
                ++c)
            if (!(*c)->isFinalized ())
            (*c)->finalize ();

            if (multiConnectors[multiId] == NULL)
              {
                std::cout << "Rank " << mpi_get_rank (MPI_COMM_WORLD)
                << " aborting on multiId " << multiId
                << " remote leader = " << connector->remoteLeader ()
                << std::endl;
                assert (0);
              }
            multiConnectors[multiId]->tick ();
            //cCache.clear ();
          }

        schedule.clear ();
        scheduler->nextCommunication (localTime, schedule);
        //scheduler->tick(localTime);
        for (std::vector<PostCommunicationConnector*>::iterator c =
            postCommunication.begin ();
            c != postCommunication.end ();
            ++c)
        (*c)->postCommunication ();
      }
    while (!cnn_ports.empty ());
#endif

#if defined (OPEN_MPI) && MPI_VERSION <= 2
    // This is needed in OpenMPI version <= 1.2 for the freeing of the
    // intercommunicators to go well
    MPI_Barrier (MPI_COMM_WORLD);
#endif
    for (std::vector<Connector*>::iterator connector = connectors.begin ();
        connector != connectors.end (); ++connector)
      (*connector)->freeIntercomm ();

    MPI_Finalize ();
  }


  void
  Runtime::tick ()
  {

    // Update local time
    localTime.tick ();
    // ContPorts do some per-tick initialization here
    std::vector<TickingPort*>::iterator p;
    for (p = tickingPorts.begin (); p != tickingPorts.end (); ++p)
      (*p)->tick ();
    // ContOutputConnectors sample data
    for (std::vector<PreCommunicationConnector*>::iterator c =
        preCommunication.begin (); c != preCommunication.end (); ++c)
      (*c)->preCommunication ();

#if 1
    scheduler->tick (localTime);
#else
    if (!schedule.empty ())
    while (schedule[0].first <= localTime.time())
      {
        std::vector< std::pair<double, Connector*> >::iterator comm;
        for (comm = schedule.begin ();
            comm != schedule.end() && comm->first <= localTime.time ();
            ++comm)
          {
            if (comm->second == NULL || comm->second->isFinalized ())
            continue;

            Connector* connector = comm->second;

            if (!connector->idFlag ())
              {
                // here standard connectors are handled
                connector->tick ();
                continue;
              }

            // all multiconnector code should go into scheduler
            // this is a temporary solution
            int remoteLeader = connector->remoteLeader ();
            bool direction = connector->isInput ();
            unsigned int multiId = connector->idFlag ();
            //cCache.push_back (connector);
            // yes, horrible quadratic algorithm for now
            for (std::vector< std::pair<double, Connector*> >::iterator c
                = comm + 1;
                c != schedule.end () && c->first <= localTime.time();
                ++c)
              {
                if (c->second != NULL
                    && c->second->idFlag ()
                    && c->second->remoteLeader () == remoteLeader
                    && c->second->isInput () == direction)
                  {
                    //cCache.push_back (c->second);
                    multiId |= c->second->idFlag ();
                    c->second = NULL;// taken
                  }
              }

            //if (mpi_get_rank (MPI_COMM_WORLD) == 2)
            //  std::cout << "multiId = " << multiId << std::endl;
            if (multiConnectors[multiId] == NULL)
              {
                assert (0);
              }
            multiConnectors[multiId]->tick ();
            //cCache.clear ();
          }
        schedule.erase (schedule.begin (), comm);
        scheduler->nextCommunication (localTime, schedule);
      }*/
#endif

    // ContInputConnectors write data to application here
    for (std::vector<PostCommunicationConnector*>::iterator c =
        postCommunication.begin (); c != postCommunication.end (); ++c)
      (*c)->postCommunication ();
  }


  double
  Runtime::time ()
  {
    return localTime.time ();
  }

}
#endif
