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

#include "music/connector.hh"

#include "music/debug.hh"

#if MUSIC_USE_MPI
// connector.hh needs to be included first since it causes inclusion
// of mpi.h (in data_map.hh).  mpi.h must be included before other
// header files on BG/L
#include "music/error.hh"
#include "music/communication.hh"
#include "music/event_router.hh"
#include <cmath>
#include <map>

namespace MUSIC {
  std::map<unsigned int, Connector*> Connector::flagMap_;

  unsigned int Connector::nextFlag_ = 1;

  unsigned int Connector::nextProxyFlag_ = 1;

  Connector::Connector (ConnectorInfo info_,
			IndexMap* indices,
			Index::Type type,
			MPI_Comm c)
    : info (info_),
      indices_ (indices->copy()),
      type_ (type),
      comm (c),
      idFlag_ (0),
      finalized_ (false)
  {
  }


  Connector::Connector (ConnectorInfo info_,
			IndexMap* indices,
			Index::Type type,
			MPI_Comm c,
			MPI_Comm ic)
    : info (info_),
      indices_ (indices->copy()),
      type_ (type),
      comm (c),
      intercomm (ic),
      idFlag_ (0),
      finalized_ (false)
  {
  }


  bool
  Connector::isLeader ()
  {
    return mpi_get_rank (comm) == 0;
  }

  
  void
  Connector::createIntercomm ()
  {

    MPI_Intercomm_create (comm,
			  0,
			  MPI_COMM_WORLD,
			  info.remoteLeader (),
			  CREATE_INTERCOMM_MSG,
			  &intercomm);
  }


  void
  Connector::freeIntercomm ()
  {
    MPI_Comm_free (&intercomm);
  }


  void Connector::spatialNegotiation ()
  {
    SpatialNegotiator *spatialNegotiator_ = createSpatialNegotiator ();
    width_ = spatialNegotiator_->getWidth ();
    maxLocalWidth_ = spatialNegotiator_->maxLocalWidth ();
    spatialNegotiation (spatialNegotiator_);
    delete spatialNegotiator_;
    delete indices_;
  }


  void Connector::spatialNegotiation (SpatialNegotiator * spatialNegotiator_)
  {

    if (spatialNegotiator_ == nullptr)
      error ("spatialNegotiator was not initialized");
    std::map<int, Subconnector*> subconnectors;
    NegotiationIterator i
      = spatialNegotiator_->negotiate (info.nProcesses (),
				       this); // only for debugging

    for (	 ;!i.end ();	 ++i)
      {
  	std::map<int, Subconnector*>::iterator c
  	  = subconnectors.find (i->rank ());
  	Subconnector* subconn;
  	if (c != subconnectors.end ())
  	  subconn = c->second;
  	else
  	  {
  	    subconn = makeSubconnector (i->rank ());
  	    if(subconn == nullptr)
	      error ("Subconnector was not initialized");
  	    subconnectors.insert (std::make_pair (i->rank (), subconn));
  	    rsubconn.push_back (subconn);

  	  }
  	MUSIC_LOG (mpi_get_rank (MPI_COMM_WORLD)
  		   << ": ("
  		   << i->begin () << ", "
  		   << i->end () << ", "
  		   << i->local () << ", "
  		   << i->displ() << ") -> " << i->rank ());
  	addRoutingInterval (i->interval (), subconn);


      }

  }


  // The following ordering of subconnectors leads to a reasonably
  // efficient communication schedule where the number of
  // communications that weill have to happen sequentially is O(P)
  // where P is the number of processors.
  //
  // Mikael Djurfeldt et al. (2005) "Massively parallel simulation
  // of brain-scale neuronal network models." Tech. Rep.
  // TRITA-NA-P0513, CSC, KTH, Stockholm.

  bool
  lessInputSubconnector (const Subconnector* c1, const Subconnector* c2)
  {
    return (c1->remoteRank () < c2->remoteRank ()
  	    || (c1->remoteRank () == c2->remoteRank ()
  		&& c1->receiverPortCode () < c2->receiverPortCode ()));
  }


  bool
  lessOutputSubconnector (const Subconnector* c1, const Subconnector* c2)
  {
    if ((c1->remoteRank () > c1->localRank ()
	 && c2->remoteRank () <= c1->localRank ())
	|| (c1->remoteRank () <= c1->localRank ()
	    && c2->remoteRank () > c1->localRank ()))
      return c1->remoteRank () > c2->remoteRank ();
    else
      return lessInputSubconnector (c1, c2);
  }


  void Connector::initialize ()
  {
    sort (rsubconn.begin (),
	  rsubconn.end (),
	  isInput () ? lessInputSubconnector : lessOutputSubconnector);
  }


  void
  Connector::finalize ()
  {
    bool dataStillFlowing = false;
    for (std::vector<Subconnector*>::iterator s = rsubconn.begin (); 	 s != rsubconn.end ();  	 ++s)
      (*s)->flush(dataStillFlowing);
    finalized_ = !dataStillFlowing;
  }


  void Connector::tick ()
  {
    for (std::vector<Subconnector*>::iterator s = rsubconn.begin ();	 s != rsubconn.end ();
	 ++s){
      (*s)->maybeCommunicate();
    }
  }


  SpatialNegotiator *
  OutputConnector::createSpatialNegotiator ()
  {
    return new SpatialOutputNegotiator(indices_, type_, comm, intercomm );
  }

#if MUSIC_ISENDWAITALL
  void
  OutputConnector::tick ()
  {
	  std::vector<MPI_Request> requests;
	  for (std::vector<Subconnector*>::iterator s = rsubconn.begin ();
	       s != rsubconn.end ();
	       ++s)
	    {
	      (*s)->maybeCommunicate (requests);
	    }
	  //The spec  guarantees vectors store their elements contiguously:
	  MPI_Waitall(requests.size(), (MPI_Request *)&requests[0], MPI_STATUSES_IGNORE);
  }
#endif //MUSIC_ISENDWAITALL

  SpatialNegotiator *
  InputConnector::createSpatialNegotiator ()
  {
    return new SpatialInputNegotiator(indices_, type_, comm, intercomm);
  }


  /********************************************************************
   *
   * Cont Connectors
   *
   ********************************************************************/

  void
  ContConnector::initialize ()
  {
    Connector::initialize();
    initialCommunication();
  }


  ClockState
  ContConnector::remoteTickInterval (ClockState tickInterval)
  {
    ClockState::Serialized sRemoteTickInterval;
    if (isLeader ())
      {
	// exchange tickInterval with peer leader
	sRemoteTickInterval = tickInterval.serialize ();
	MPI_Sendrecv_replace (&sRemoteTickInterval, 2, MPI_UNSIGNED_LONG,
			      0, TICKINTERVAL_MSG,
			      0, TICKINTERVAL_MSG,
			      intercomm,
			      MPI_STATUS_IGNORE);
      }
    // broadcast to peers
    MPI_Bcast (&sRemoteTickInterval, 2, MPI_UNSIGNED_LONG, 0, comm);
    return sRemoteTickInterval.deserialize ();
  }


  ContOutputConnector::ContOutputConnector (ConnectorInfo connInfo,
					    IndexMap* indices,
					    Index::Type type,
					    MPI_Comm comm,
					    Sampler& sampler,
					    MPI_Datatype data_type)
    : Connector (connInfo, indices, type, comm),
      ContConnector (sampler, data_type),
      connector(nullptr)
  {
  }

  ContOutputConnector::ContOutputConnector (   Sampler& sampler,  MPI_Datatype type)
    :  ContConnector (sampler, type),
       connector(nullptr)
  {
  }


  ContOutputConnector::~ContOutputConnector()
  {
    if (connector != nullptr)
      delete connector;
  }


  Subconnector*
  ContOutputConnector::makeSubconnector (int remoteRank)
  {
    return new ContOutputSubconnector (//synchronizer (),
				       intercomm,
				       remoteLeader (),
				       remoteRank,
				       receiverPortCode (),
				       data_type_);
  }


  void
  ContOutputConnector::specialize (Clock& localTime)
  {
    ClockState tickInterval = localTime.tickInterval ();
    ClockState rTickInterval = remoteTickInterval (tickInterval);
    localTime_ = &localTime;
    remoteTime_.configure (localTime_->timebase (), rTickInterval);
    if (tickInterval < rTickInterval)
      connector = new InterpolatingContOutputConnector (this);
    else
      connector = new PlainContOutputConnector (this);
  }

  
  void
  ContOutputConnector::addRoutingInterval (IndexInterval i,
					   Subconnector* subconn)
  {
    OutputSubconnector*osubconn= dynamic_cast<OutputSubconnector*>(subconn);
    distributor_.addRoutingInterval (i, osubconn->outputBuffer());
  }


  void ContOutputConnector::initialCommunication()
  {
    for (std::vector<Subconnector*>::iterator s = rsubconn.begin (); s != rsubconn.end (); ++s)
      (*s)->initialCommunication (0.0);
  }
  

  void
  PlainContOutputConnector::initialize ()
  {
    distributor_.configure (sampler_.dataMap ());
    distributor_.initialize ();
    // synch.initialize ();

    // put one element in send buffers
    distributor_.distribute ();
  }


  void
  PlainContOutputConnector::preCommunication ()
  {
    if (sample ())
      {
	// copy application data to send buffers
	distributor_.distribute ();

      }
  }


  // Start sampling (and fill the output buffers) at a time dependent
  // on latency and receiver's tick interval.  A negative latency can
  // delay start of sampling beyond time 0.  The tickInterval together
  // with the strict comparison has the purpose of supplying an
  // interpolating receiver side with samples.
  bool
  PlainContOutputConnector::sample  ()
  {
    return (localTime_->integerTime () + latency + remoteTime_.tickInterval ()
	    > 0);
  }


  void
  InterpolatingContOutputConnector::initialize ()
  {
    // Set the remoteTime which is used to control sampling and
    // interpolation.
    //
    // For positive latencies, the integer part (in terms of receiver
    // ticks) of the latency is handled by filling up the receiver
    // buffers using InputSynchronizer::initialBufferedTicks ().
    // remoteTime then holds the fractional part.
    if (latency > 0)
      {
	ClockState startTime = - latency % remoteTime_.tickInterval ();
	if (latency >= localTime_->tickInterval ())
	  startTime = startTime + remoteTime_.tickInterval ();
	remoteTime_.set (startTime);
      }
    else
      remoteTime_.set (- latency);
    distributor_.configure (sampler_.interpolationDataMap ());
    distributor_.initialize ();
    // synch.initialize ();

    // put one element in send buffers
    sampler_.sample ();
    sampler_.interpolate (1.0);
    distributor_.distribute ();
  }


  // After the last sample at 1 localTime will be between remoteTime
  // and remoteTime + localTime->tickInterval.  We trigger on this
  // situation and forward remoteTime.
  bool
  InterpolatingContOutputConnector::sample()
  {
    ClockState sampleWindowLow
      = remoteTime_.integerTime () - localTime_->tickInterval ();
    ClockState sampleWindowHigh
      = remoteTime_.integerTime () + localTime_->tickInterval ();
    return (sampleWindowLow <= localTime_->integerTime ()
	    && localTime_->integerTime () < sampleWindowHigh);
  }


  bool
  InterpolatingContOutputConnector::interpolate()
  {
    ClockState sampleWindowHigh
      = remoteTime_.integerTime () + localTime_->tickInterval ();
    return (remoteTime_.integerTime () <= localTime_->integerTime ()
	    && localTime_->integerTime () < sampleWindowHigh);
  }


  double
  InterpolatingContOutputConnector::interpolationCoefficient()
  {
    ClockState prevSampleTime
      = localTime_->integerTime () - localTime_->tickInterval ();
    double c = ((double) (remoteTime_.integerTime () - prevSampleTime)
		/ (double) localTime_->tickInterval ());

    MUSIC_LOGR ("interpolationCoefficient = " << c);
    // NOTE: preliminary implementation which just provides
    // the functionality specified in the API
    if (interp)
      return c;
    else
      return round (c);
  }


  void
  InterpolatingContOutputConnector::preCommunication ()
  {
    if (sample ()){
      // sampling before and after time of receiver tick
      sampler_.sampleOnce ();
    }
    if (interpolate ())
      {
	sampler_.interpolate (interpolationCoefficient ());
	remoteTime_.tick();
	distributor_.distribute ();
      }
  }


  ContInputConnector::ContInputConnector (ConnectorInfo connInfo,
		  	  	  	  IndexMap* indices,
		  	  	  	  Index::Type type,
					  MPI_Comm comm,
					  Sampler& sampler,
					  MPI_Datatype data_type,
					  double delay)
    : Connector (connInfo, indices, type, comm),
      ContConnector (sampler, data_type),
      delay_ (delay),
      connector(nullptr)
  {
  }


  ContInputConnector::ContInputConnector ( Sampler& sampler, MPI_Datatype type,  double delay)
    : ContConnector (sampler, type),
      delay_ (delay),
      connector(nullptr)
  {
  }


  ContInputConnector::~ContInputConnector()
  {
    if (connector != nullptr)
      delete connector;
  }


  Subconnector*
  ContInputConnector::makeSubconnector (int remoteRank)
  {
    int receiverRank = mpi_get_rank (intercomm);
    return new ContInputSubconnector (//synchronizer (),
				      intercomm,
				      remoteLeader (),
				      remoteRank,
				      receiverRank,
				      receiverPortCode (),
				      data_type_);
  }


  bool
  ContInputConnector::divisibleDelay (Clock& localTime)
  {
    ClockState delay (delay_, localTime.timebase ());
    return (delay % localTime.tickInterval ()) == 0;
  }
  

  void
  ContInputConnector::specialize (Clock& localTime)
  {
    ClockState tickInterval = localTime.tickInterval ();
    ClockState rTickInterval = remoteTickInterval (tickInterval);
    localTime_ = &localTime;
    remoteTime_.configure (localTime_->timebase (), rTickInterval);

    if (tickInterval < rTickInterval
	|| (tickInterval == rTickInterval
	    && !divisibleDelay (localTime)))
      connector = new InterpolatingContInputConnector(this);
    else
      connector = new PlainContInputConnector (this);
  }


  void
  ContInputConnector::initialCommunication()
  {
    double initialBTicks = initialBufferedTicks();
    for (std::vector<Subconnector*>::iterator s = rsubconn.begin (); s != rsubconn.end (); ++s)
      (*s)->initialCommunication (initialBTicks);
  }


  // Return the number of copies of the data sampled by the sender
  // Runtime constructor which should be stored in the receiver
  // buffers at the first tick () (which occurs at the end of the
  // Runtime constructor)
  int
  ContInputConnector::initialBufferedTicks ()
  {
    if (remoteTime_.tickInterval () < localTime_->tickInterval ())
      {
	// InterpolatingOutputConnector - PlainInputConnector

	if (delay_ <= 0)
	  return 0;
	else
	  {
	    // Need to add a sample first when we pass the receiver
	    // tick (=> - 1).  If we haven't passed, the interpolator
	    // could simply use an interpolation coefficient of 0.0.
	    // (But this will never happen since that case isn't
	    // handled by an InterpolatingOutputConnector.)
	    int ticks = (delay_ - 1) / localTime_->tickInterval ();

	    // Need to add a sample if we go outside of the sender
	    // interpolation window
	    if (delay_ >= remoteTime_.tickInterval ())
	      ticks += 1;

	    return ticks;
	  }
      }
    else
      {
	// PlainOutputConnector - InterpolatingInputConnector

	if (delay_ <= 0)
	  return 0;
	else
	  // Need to add a sample first when we pass the receiver
	  // tick (=> - 1).  If we haven't passed, the interpolator
	  // can simply use an interpolation coefficient of 0.0.
	  return 1 + (delay_ - 1) / localTime_->tickInterval ();
      }
  }


  void
  ContInputConnector::addRoutingInterval (IndexInterval i,
					  Subconnector* subconn)
  {
    InputSubconnector*isubconn= dynamic_cast<InputSubconnector*>(subconn);
    collector_.addRoutingInterval (i, isubconn->inputBuffer());
  }


  void
  PlainContInputConnector::initialize ()
  {
    // collector_.configure (sampler_.dataMap (), synch.allowedBuffered () + 1);
    collector_.configure (sampler_.dataMap (),  CONT_BUFFER_MAX);
    collector_.initialize ();
  }
  

  void
  PlainContInputConnector::postCommunication ()
  {
    // collect data from input buffers and write to application
    collector_.collect ();
  }


  void
  InterpolatingContInputConnector::initialize ()
  {
    if (latency > 0)
      remoteTime_.set (latency % remoteTime_.tickInterval ()
		       - 2 * remoteTime_.tickInterval ());
    else
      remoteTime_.set (latency % remoteTime_.tickInterval ()
		       - remoteTime_.tickInterval ());
    //  collector_.configure (sampler_.interpolationDataMap (),
    //			  synch.allowedBuffered () + 1);

    collector_.configure (sampler_.interpolationDataMap (), CONT_BUFFER_MAX);
    collector_.initialize ();


    // synch.initialize ();
  }


  bool
  InterpolatingContInputConnector::sample ()
  {
    return localTime_->integerTime () > remoteTime_.integerTime ();
  }


  double
  InterpolatingContInputConnector::interpolationCoefficient ()
  {
    ClockState prevSampleTime
      = remoteTime_.integerTime () - remoteTime_.tickInterval ();
    double c = ((double) (localTime_->integerTime () - prevSampleTime)
		/ (double) remoteTime_.tickInterval ());

    MUSIC_LOGR ("interpolationCoefficient = " << c);
    // NOTE: preliminary implementation which just provides
    // the functionality specified in the API
    if (interp)
      return c;
    else
      return round (c);
  }


  void
  InterpolatingContInputConnector::postCommunication ()
  {
    if (first_)
      {
	collector_.collect (sampler_.insert ());
	remoteTime_.tick();
	first_ = false;
      }
    else if (sample ())
      {
	collector_.collect (sampler_.insert ());
	remoteTime_.tick();
      }
    sampler_.interpolateToApplication (interpolationCoefficient ());
  }


  /********************************************************************
   *
   * Event Connectors
   *
   ********************************************************************/


  Subconnector*
  EventOutputConnector::makeSubconnector (int remoteRank)
  {
    //////
/*              int world_size = mpi_get_comm_size (MPI_COMM_WORLD);
                if (mpi_get_rank (MPI_COMM_WORLD) < world_size/2)
                  std::cout << remoteRank << std::flush << std::endl;*/
    return new EventOutputSubconnector (//&synch,
					intercomm,
					remoteLeader (),
					remoteRank,
					receiverPortCode ());
  }


  void
  EventOutputConnector::addRoutingInterval (IndexInterval i,
					    Subconnector* subconn)
  {
    OutputSubconnector* osubconn= dynamic_cast<OutputSubconnector*>(subconn);
    routingMap_-> insert (i, osubconn->outputBuffer());
  }

  Subconnector*
  EventInputConnector::makeSubconnector (int remoteRank)
  {
    int receiverRank = mpi_get_rank (intercomm);
    if (type_ == Index::GLOBAL){
    	EventInputSubconnectorGlobal *subcc =
    	 new EventInputSubconnectorGlobal (//&synch,
					       intercomm,
					       remoteLeader (),
					       remoteRank,
					       receiverRank,
					       receiverPortCode (),
					       handleEvent_);
    	rRank2Subconnector[remoteRank] = subcc;
    	flushes++;
    	return subcc;
    }
    else
      {
#if MUSIC_ANYSOURCE
error( "LOCAL Indices are not supported with MUSIC_ANYSOURCE");
#endif

      return new EventInputSubconnectorLocal (//&synch,
					      intercomm,
					      remoteLeader (),
					      remoteRank,
					      receiverRank,
					      receiverPortCode (),
					      handleEvent_);
      }
  }



  void
  EventInputConnector::addRoutingInterval(IndexInterval i, Subconnector* subconn)
  {
    //routingMap_-> insert (i, handleEvent_.global());
  }
#if MUSIC_ANYSOURCE
 void
 EventInputConnector::tick ()
   {
	  //SPIKE_BUFFER_MAX size
	  //MPI_BYTE type
	  //SPIKE_MSG tag
	  int size = rsubconn.size();
	  char data[SPIKE_BUFFER_MAX];
	  MPI_Status status;
	  while(size > 0 && flushes > 0){
	  MPI_Recv (data,
		    SPIKE_BUFFER_MAX,
		    MPI_BYTE,
		    MPI_ANY_SOURCE,
		    SPIKE_MSG,
		    intercomm,
		    &status);

	  int msize;
	  MPI_Get_count (&status, MPI_BYTE, &msize);
	  if (rRank2Subconnector[status.MPI_SOURCE]->receive (data, msize))
	    flushes--;
	  if( msize < SPIKE_BUFFER_MAX)
		 size--;
	  }
   }
#endif //MUSIC_ANYSOURCE


  /********************************************************************
   *
   * Message Connectors
   *
   ********************************************************************/
  
  MessageOutputConnector::MessageOutputConnector (ConnectorInfo connInfo,
		  	  	  	  	  IndexMap* indices,
		  	  	  	  	  Index::Type type,
						  MPI_Comm comm,
						  std::vector<FIBO*>& buffers)
    : Connector (connInfo, indices, type, comm),
      buffer (1),
      bufferAdded (false),
      buffers_ (buffers)
  {
  }

  
  Subconnector*
  MessageOutputConnector::makeSubconnector (int remoteRank)
  {
    return new MessageOutputSubconnector (//&synch,
					  intercomm,
					  remoteLeader (),
					  remoteRank,
					  receiverPortCode (),
					  &buffer);
  }


  void
  MessageOutputConnector::addRoutingInterval (IndexInterval i,
					      Subconnector* subconn)
  {
    if (!bufferAdded)
      {
	buffers_.push_back (&buffer);
	bufferAdded = true;
      }
  }
  
  


  
  void
  MessageOutputConnector::postCommunication ()
  {
  }

  
  MessageInputConnector::MessageInputConnector (ConnectorInfo connInfo,
						IndexMap* indices,
						Index::Type type,
						MessageHandler* handleMessage,
						MPI_Comm comm)
    : Connector (connInfo, indices, type, comm),
      handleMessage_ (handleMessage)
  {
  }



  
  Subconnector*
  MessageInputConnector::makeSubconnector (int remoteRank)
  {
    int receiverRank = mpi_get_rank (intercomm);
    return new MessageInputSubconnector (//&synch,
					 intercomm,
					 remoteLeader (),
					 remoteRank,
					 receiverRank,
					 receiverPortCode (),
					 handleMessage_);
  }

  /********************************************************************
   *
   * Collective Connector
   *
   ********************************************************************/

  CollectiveConnector::CollectiveConnector (bool high)
    : high_ (high), subconnector_ (nullptr)
  {
    //idFlag_ = makeFlag ();
  }


  void
  CollectiveConnector::createIntercomm ()
  {
    Connector::createIntercomm ();
    MPI_Intercomm_merge (intercomm, high_, &intracomm_);
  }


  void
  CollectiveConnector::freeIntercomm ()
  {
    MPI_Comm_free (&intracomm_);
    Connector::freeIntercomm ();
  }


  ContCollectiveConnector::ContCollectiveConnector (MPI_Datatype type,
						    bool high):
    CollectiveConnector (high),
    data_type (type)
  {

  }


  Subconnector*
  ContCollectiveConnector::makeSubconnector (void *param)
  {
    std::multimap< int, Interval> intrvs
      = *static_cast< std::multimap< int, Interval>*> (param);
    if (subconnector_ == nullptr)
      {
	subconnector_ = new ContCollectiveSubconnector (intrvs,
							width (),
							intracomm_,
							data_type);
      }
    return subconnector_;
  }


  EventCollectiveConnector::EventCollectiveConnector (bool high)
    : CollectiveConnector (high), router_ (nullptr)
  {
    idFlag_ = makeFlag (this);
  }


  Subconnector*
  EventInputCollectiveConnector::makeSubconnector (void *param)
  {
    if (subconnector_ == nullptr)
      subconnector_ = new EventInputCollectiveSubconnector (router_);
    return subconnector_;
  }


  EventInputCollectiveConnector::EventInputCollectiveConnector
  (ConnectorInfo connInfo,
   IndexMap* indices,
   Index::Type type,
   MPI_Comm comm,
   EventHandlerPtr handleEvent)
    : Connector (connInfo, indices, type, comm),
      EventInputConnector (handleEvent),
      EventCollectiveConnector (true)
  {

    int procMethod = connInfo.processingMethod ();
    if (procMethod == ConnectorInfo::TREE)
      if (handleEvent.getType () == Index::GLOBAL)
	router_ = new TreeProcessingInputGlobalRouter ();
      else
	router_ = new TreeProcessingInputLocalRouter ();
    else
      if (handleEvent.getType () == Index::GLOBAL)
	router_ = new TableProcessingInputGlobalRouter ();
      else
	router_ = new TableProcessingInputLocalRouter ();
  }


  EventInputCollectiveConnector::~EventInputCollectiveConnector()
  {
    delete router_;
  }


  void
  EventInputCollectiveConnector::spatialNegotiation (SpatialNegotiator* spatialNegotiator_)
  {
    routingMap_input = new InputRoutingMap ();
    Subconnector* subconn
      = EventInputCollectiveConnector::makeSubconnector (nullptr);
    rsubconn.push_back (subconn);
    for (NegotiationIterator i = spatialNegotiator_->negotiateSimple (); !i.end (); ++i)
      addRoutingInterval (i->interval (), subconn);
    routingMap_input->fillRouter (router_);
    delete routingMap_input;
    router_->buildTable ();
  }


  void
  EventInputCollectiveConnector::addRoutingInterval (IndexInterval i,
						     Subconnector* subconn)
  {
    routingMap_input->insert (i, &handleEvent_);
  }


  EventOutputCollectiveConnector::EventOutputCollectiveConnector
  (ConnectorInfo connInfo,
   IndexMap* indices,
   Index::Type type,
   MPI_Comm comm,
   DirectRouter* router)
    : Connector (connInfo, indices, type, comm),
      EventOutputConnector (nullptr),
      EventCollectiveConnector (false),
      directRouter_ (router)
  {
  }


  EventOutputCollectiveConnector::~EventOutputCollectiveConnector()
  {
  }


  Subconnector*
  EventOutputCollectiveConnector::makeSubconnector (void *param)
  {
    //  EventRouter * router_ = static_cast<EventRouter*>(param);
    if(subconnector_ == nullptr){
      subconnector_ = new EventOutputCollectiveSubconnector ();
    }
    return subconnector_;
  }

  void
  EventOutputCollectiveConnector::spatialNegotiation (SpatialNegotiator* spatialNegotiator_)
  {
    EventOutputCollectiveSubconnector* subconn
      = dynamic_cast<EventOutputCollectiveSubconnector*>
      (EventOutputCollectiveConnector::makeSubconnector (nullptr));
    rsubconn.push_back (subconn);
    subconn->setRouter (directRouter_);
    //spatialNegotiator_->negotiateSimple ();
  }


  ContInputCollectiveConnector::ContInputCollectiveConnector
  (ConnectorInfo connInfo,
   IndexMap* indices,
   Index::Type type,
   MPI_Comm comm,
   Sampler& sampler,
   MPI_Datatype data_type,
   double delay)
    : Connector(connInfo, indices,type, comm),
      ContInputConnector(sampler, data_type, delay),
      ContCollectiveConnector(data_type, true)
  {

  }


  void
  ContInputCollectiveConnector::spatialNegotiation (SpatialNegotiator* spatialNegotiator_)
  {
    std::multimap< int, Interval> receiver_intrvs;
    std::vector<IndexInterval> intrvs;
    std::map<int,int> remoteToCollectiveRankMap;
    receiveRemoteCommRankID(remoteToCollectiveRankMap);

    for (NegotiationIterator i  = spatialNegotiator_->negotiate (info.nProcesses (), this);  !i.end (); ++i)
      {
	int begin = (i->displ()) * mpi_get_type_size (data_type_);
	// length field is stored overlapping the end field
	int end = (i->end() - i->begin()) * mpi_get_type_size (data_type_);
	receiver_intrvs.insert (std::make_pair ( remoteToCollectiveRankMap[i->rank ()], Interval(begin, end)));
	intrvs.push_back(i->interval());
      }

    Subconnector *subconn = ContCollectiveConnector::makeSubconnector(&receiver_intrvs);
    rsubconn.push_back (subconn);
    for (std::vector<IndexInterval>::iterator  it=intrvs.begin() ; it < intrvs.end(); it++ )
      addRoutingInterval ((*it), subconn);
  }


  void
  ContInputCollectiveConnector::receiveRemoteCommRankID(std::map<int,int> &remoteToCollectiveRankMap)
  {
    int nProcesses, intra_rank;
    MPI_Comm_remote_size (intercomm, &nProcesses);

    for (int i =0; i < nProcesses; ++i)
      {
	MPI_Recv (&intra_rank,
		  1,
		  MPI_INT,
		  i,
		  SPATIAL_NEGOTIATION_MSG,
		  intercomm,
		  MPI_STATUS_IGNORE);
	remoteToCollectiveRankMap.insert(std::make_pair(i,intra_rank));
	MUSIC_LOG0( "Remote Communication Rank:" << i << "is mapped to Collective Communication Rank:" << intra_rank );
      }

  }


  ContOutputCollectiveConnector::ContOutputCollectiveConnector(ConnectorInfo connInfo,
							       IndexMap* indices,
							       Index::Type type,
							       MPI_Comm comm,
							       Sampler& sampler,
							       MPI_Datatype data_type):
    Connector(connInfo, indices,type, comm),
    ContOutputConnector( sampler, data_type),
    ContCollectiveConnector(data_type, false)
  {

  }


  void
  ContOutputCollectiveConnector::spatialNegotiation ( SpatialNegotiator* spatialNegotiator_)
  {
    sendLocalCommRankID();
    std::map<Interval, int>  empty_intrvs;
    Subconnector *subconn = ContCollectiveConnector::makeSubconnector(&empty_intrvs);
    rsubconn.push_back (subconn);
    spatialNegotiator_->negotiate (info.nProcesses (), this);
    for (NegotiationIterator i = spatialNegotiator_->negotiateSimple (); !i.end (); ++i)
      addRoutingInterval (i->interval(), subconn);
  }


  void
  ContOutputCollectiveConnector::sendLocalCommRankID()
  {
    int nProcesses, intra_rank;
    MPI_Comm_remote_size (intercomm, &nProcesses);
    intra_rank = mpi_get_rank (intracomm_);
    std::map<int,int> rCommToCollCommRankMap;
    for (int i =0; i < nProcesses; ++i){
      MPI_Ssend (&intra_rank,
		 1,
		 MPI_INT,
		 i,
		 SPATIAL_NEGOTIATION_MSG,
		 intercomm);
    }
  }
}
#endif
