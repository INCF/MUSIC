/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007-2012 INCF
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

  Runtime::Runtime (const Runtime& other)
	  : app_ (other.app_),
	  portMgr_ (other.portMgr_),
	  localTime (other.localTime),
	  // here: new temporalNegotiator
	  temporalNegotiator_ (other.app_),
	  leader_ (other.leader_),
	  comm (other.comm),
	  scheduler (new Scheduler (comm, leader_)),
	  mAgent (0)
  {
	setup ();
  }

  Runtime::Runtime (Application& app, PortConnectivityManager& portMgr, double h)
    : app_ (app),
	portMgr_ (portMgr),
	localTime (Clock (app_.timebase (), h)),
	temporalNegotiator_ (app_),
	leader_ (app_.leader ()),
	comm (app_.communicator ()),
	scheduler (new Scheduler (comm, leader_)),
	mAgent (0),
  {
	setup ();
  }


  // TODO passing connections directly to setup() instead of extracting them
  // inside
  void Runtime::setup ()
  {
	Connections* connections = new Connections ();
	auto& ports = portMgr_.getPorts ();

	std::for_each (ports.begin (), ports.end (),
			[connections](auto& port_ptr)
			{
				connections->push_back (port_ptr->connections_);
			});

    if (app_.launchedByMusic ())
      {
        takeTickingPorts ();

        // create a total order for connectors and
        // establish connection to peers
        connectToPeers (connections);

        // specialize connectors and fill up connectors vector, specialization :
		// { Input/OutputConnector whether ticking or plain}
        specializeConnectors (connections);
        // from here we can start using the vector `connectors'
        // negotiate where to route data and fill up subconnector vectors
        spatialNegotiation ();

        // build data routing tables
        buildTables ();
        takePreCommunicators ();
        takePostCommunicators ();
        // negotiate timing constraints for synchronizers
        temporalNegotiation (connections);
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
        initialize ();
      }
    else
      {
	sAgents.push_back (new DummyAgent (scheduler));
	scheduler->setAgent (sAgents[0]);
	scheduler->initializeAgentState ();
      }

    delete s;
#ifdef MUSIC_AFTER_RUNTIME_CONSTRUCTOR_REPORT
    if (MPI::COMM_WORLD.Get_rank () == 0)
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
    /* for (std::vector<Connector*>::iterator connector = connectors.begin (); */
    /*     connector != connectors.end (); ++connector) */
    /*   delete *connector; */
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
  Runtime::takeTickingPorts ()
  {
	SPVec<Port>::iterator p;
    for (p = ports.begin (); p != ports.end (); ++p)
      {
        TickingPort* tp = dynamic_cast<TickingPort*> (*p);
		std::shared_ptr<TickingPort> tp (std::dynamic_pointer_cast<TickingPort> (port));
        if (tp != nullptr)
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
  Runtime::buildTables ()
  {
    for (std::vector<Port*>::iterator p = ports.begin (); p != ports.end ();
        ++p)
      (*p)->buildTable ();
  }


  void
  Runtime::temporalNegotiation (Connections* connections)
  {
    // Temporal negotiation is done globally by a serial algorithm
    // which yields the same result in each process
    temporalNegotiator_.negotiate (localTime, connections);
  }


  MPI::Intracomm
  Runtime::communicator ()
  {
    return comm;
  }


  void
  Runtime::initialize ()
  {
    scheduler->initialize (temporalNegotiator_.applicationGraph (),
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
#endif

#if defined (OPEN_MPI) && MPI_VERSION <= 2
    // This is needed in OpenMPI version <= 1.2 for the freeing of the
    // intercommunicators to go well
    MPI::COMM_WORLD.Barrier ();
#endif
    for (std::vector<Connector*>::iterator connector = connectors.begin ();
        connector != connectors.end (); ++connector)
      (*connector)->freeIntercomm ();

    MPI::Finalize ();
  }


  void
  Runtime::tick ()
  {

    // Update local time
    localTime.tick ();
    // ContPorts do some per-tick initialization here
	SPVec<TickingPort>::iterator p;
    for (p = tickingPorts.begin (); p != tickingPorts.end (); ++p)
      (*p)->tick ();
    // ContOutputConnectors sample data
    for (std::vector<PreCommunicationConnector*>::iterator c =
        preCommunication.begin (); c != preCommunication.end (); ++c)
      (*c)->preCommunication ();

#if 1
    scheduler->tick (localTime);
#else
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
