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
#include "music/port.hh"

#if MUSIC_USE_MPI
#include "music/event_router.hh"

#include "music/setup.hh" // Must be included first on BG/L
#include "music/error.hh"

namespace MUSIC {

  Port::Port (Setup* s, std::string identifier)
    : portName_ (identifier), setup_ (s), isMapped_ (false)
  {
    s->maybePostponedSetup ();
    ConnectivityInfo_ = s->portConnectivity (portName_);
    setup_->addPort (this);
  }


  bool
  Port::isConnected ()
  {
    return ConnectivityInfo_ != Connectivity::NO_CONNECTIVITY;
  }


  void
  Port::checkConnected (std::string action)
  {
    if (!isConnected ())
      {
	std::ostringstream msg;
	msg << "attempt to " << action << " port `" << portName_
	    << "' which is unconnected";
	error (msg);
      }
  }
  
  
  void
  Port::assertOutput ()
  {
    checkCalledOnce (isMapped_,
		     "OutputPort::map (...)",
		     " for port " + portName_);
    checkConnected ("map");
    if (ConnectivityInfo_->direction () != ConnectivityInfo::OUTPUT)
      {
	std::ostringstream msg;
	msg << "output port `" << ConnectivityInfo_->portName ()
	    << "' connected as input";
	error (msg);
      }
  }


  void
  Port::assertInput ()
  {
    checkCalledOnce (isMapped_,
		     "InputPort::map (...)",
		     " for port " + portName_);
    checkConnected ("map");
    if (ConnectivityInfo_->direction () != ConnectivityInfo::INPUT)
      {
	std::ostringstream msg;
	msg << "input port `" << ConnectivityInfo_->portName ()
	    << "' connected as output";
	error (msg);
      }
  }


  bool
  Port::hasWidth ()
  {
    checkConnected ("ask for width of");
    return ConnectivityInfo_->width () != ConnectivityInfo::NO_WIDTH;
  }


  int
  Port::width ()
  {
    checkConnected ("ask for width of");
    int w = ConnectivityInfo_->width ();
    if (w == ConnectivityInfo::NO_WIDTH)
      {
	std::ostringstream msg;
	msg << "width requested for port `" << ConnectivityInfo_->portName ()
	    << "' which has unspecified width";
	error (msg);
      }
    return w;
  }


  void
  Port::checkIndexMap (IndexMap* indexMap)
  {
    if (indexMap->begin () != indexMap->end ())
      {
	IndexMap::iterator i = indexMap->begin ();
	IndexInterval last = *i;
	++i;
	for (; i != indexMap->end (); ++i)
	  {
	    if (i->begin () < last.end ())
	      {
		std::ostringstream os;
		os << "Mapping port " << portName_
		   << " using index map with overlaps: ["
		   << last.begin () << ", " << last.end () << "), ["
		   << i->begin () << ", " << i->end () << ")";
		error (os);
	      }
	    last = *i;
	  }
      }
  }


  void
  OutputPort::mapImpl (IndexMap* indices,
		       Index::Type type,
		       int maxBuffered,
		       int dataSize)
  {
    checkIndexMap (indices);
    // The timing algorithm in uses a different definition of
    // maxBuffered.  While the interface in port.hh counts the number
    // of "ticks of data" which must be stored, the timing algorithm
    // counts the offset in time between the processes in sender
    // ticks.
    if (maxBuffered != MAX_BUFFERED_NO_VALUE)
      maxBuffered -= 1;
	
    // Retrieve info about all remote connectors of this port
    PortConnectorInfo portConnections
      = ConnectivityInfo_->connections ();
    indices_ = indices;
    index_type_ = type;
    for (PortConnectorInfo::iterator info = portConnections.begin ();
	 info != portConnections.end ();
	 ++info)
      {
	// Create connector
	Connector* connector = makeConnector (*info);
	setup_->addConnection (new OutputConnection (connector,
						     maxBuffered,
						     dataSize));
      }
  }



  
  void
  InputPort::mapImpl (IndexMap* indices,
		      Index::Type type,
		      double accLatency,
		      int maxBuffered,
		      bool interpolate)
  {
    checkIndexMap (indices);
    // The timing algorithm in uses a different definition of
    // maxBuffered.  While the interface in port.hh counts the number
    // of "ticks of data" which must be stored, the timing algorithm
    // counts the offset in time between the processes in sender
    // ticks.
    if (maxBuffered != MAX_BUFFERED_NO_VALUE)
      maxBuffered -= 1;
	
    // Retrieve info about all remote connectors of this port
    PortConnectorInfo portConnections
      = ConnectivityInfo_->connections ();
    PortConnectorInfo::iterator info = portConnections.begin ();
    indices_ = indices;
    index_type_ = type;
    Connector* connector = makeConnector (*info);
    ClockState integerLatency (accLatency, setup_->timebase ());
    setup_->addConnection (new InputConnection (connector,
						maxBuffered,
						integerLatency,
						interpolate));
  }

  
  /********************************************************************
   *
   * Cont Ports
   *
   ********************************************************************/
  
  void
  ContOutputPort::map (DataMap* dmap)
  {
    assertOutput ();
    int maxBuffered = MAX_BUFFERED_NO_VALUE;
    mapImpl (dmap, maxBuffered);
  }

  
  void
  ContOutputPort::map (DataMap* dmap, int maxBuffered)
  {
    assertOutput ();
    if (maxBuffered <= 0)
      {
	error ("ContOutputPort::map: maxBuffered should be a positive integer");
      }
    mapImpl (dmap, maxBuffered);
  }

  
  void
  ContOutputPort::mapImpl (DataMap* dmap,
			   int maxBuffered)
  {
    sampler.configure (dmap);
    type_ = dmap->type ();
    OutputPort::mapImpl (dmap->indexMap (),
				       Index::GLOBAL,
				       maxBuffered,
				       0);
  }


  Connector*
  ContOutputPort::makeConnector (ConnectorInfo connInfo)
  {
	  Connector * conn;
	  if(connInfo.communicationType() ==  ConnectorInfo::POINTTOPOINT){
		  conn =  new ContOutputConnector (connInfo,
				  indices_,
		  		  index_type_,
				  setup_->communicator (),
				  sampler,
				  type_);
	  }
	  else
		  conn = new ContOutputCollectiveConnector(connInfo,
				  indices_,
		  		  index_type_,
				  setup_->communicator (),
				  sampler,
				  type_);

	  return conn;
  }
  
  
  void
  ContOutputPort::tick ()
  {
    sampler.newSample ();
  }


  void
  ContInputPort::map (DataMap* dmap, double delay, bool interpolate)
  {
    assertInput ();
    int maxBuffered = MAX_BUFFERED_NO_VALUE;
    mapImpl (dmap,
	     delay,
	     maxBuffered,
	     interpolate);
  }

  
  void
  ContInputPort::map (DataMap* dmap,
		      int maxBuffered,
		      bool interpolate)
  {
    assertInput ();
    mapImpl (dmap,
	     0.0,
	     maxBuffered,
	     interpolate);
  }

  
  void
  ContInputPort::map (DataMap* dmap,
		      double delay,
		      int maxBuffered,
		      bool interpolate)
  {
    assertInput ();
    mapImpl (dmap,
	     delay,
	     maxBuffered,
	     interpolate);
  }

  
  void
  ContInputPort::mapImpl (DataMap* dmap,
			  double delay,
			  int maxBuffered,
			  bool interpolate)
  {
    sampler.configure (dmap);
    delay_ = delay;
    type_ = dmap->type ();
    InputPort::mapImpl (dmap->indexMap (),
				      Index::GLOBAL,
				      delay,
				      maxBuffered,
				      interpolate);
  }

  
  Connector*
  ContInputPort::makeConnector (ConnectorInfo connInfo)
  {
	  Connector * conn;
	  if(connInfo.communicationType() ==  ConnectorInfo::POINTTOPOINT){
		  conn = new ContInputConnector (connInfo,
				  indices_,
		  		  index_type_,
				  setup_->communicator (),
				  sampler,
				  type_,
				  delay_);
	  }
	  else
		  conn = new ContInputCollectiveConnector (connInfo,
				  indices_,
		  		  index_type_,
				  setup_->communicator (),
				  sampler,
				  type_,
				  delay_);
	  return conn;
  }

  
  /********************************************************************
   *
   * Event Ports
   *
   ********************************************************************/

  EventOutputPort::EventOutputPort (Setup* s, std::string id)
    : Port (s, id), routingMap (new OutputRoutingMap () /* deleted in buildTable */)
  {
    /* remedius
     * Depending on the communication type (<commType>) and
     * processing method (<procMethod>) that was introduced as
     * runtime configuration options,
     * particular processing router should be created on the output side.
     * When collective communication type is used,
     * then the processing method has to be TABLE on the output side, as in this case
     * the processing on the output side means just an insertion of the event to the buffer.
     * The difference between point-to-point and collective communication types is that on the output side
     * in the latest case we have only one common buffer (one CollectiveSubconnector) and processing is happening on the receiver side.
     */
    if (isConnected ())
      {
	bool mixed = false;
	PortConnectorInfo::iterator c = ConnectivityInfo_->connections ().begin ();
	int commType = c->communicationType ();
	int procMethod = c->processingMethod ();
	for (++c; c != ConnectivityInfo_->connections ().end (); ++c)
	  {
	    if (c->processingMethod () != procMethod)
	      error ("MUSIC: can't use both tree and table for same port");
	    if (c->communicationType () != commType)
	      mixed = true;
	  }
	if (!mixed)
	  {
	    if (commType == ConnectorInfo::COLLECTIVE)
	      router = new DirectRouter ();
	    else if (procMethod == ConnectorInfo::TREE)
	      router = new TreeProcessingOutputRouter ();
	    else
	      router = new TableProcessingOutputRouter ();
	  }
	else
	  {
	    if (procMethod == ConnectorInfo::TREE)
	      router = new HybridTreeProcessingOutputRouter ();
	    else
	      router = new HybridTableProcessingOutputRouter ();
	  }
      }
  }


  EventOutputPort::~EventOutputPort()
 {
   if (router != NULL)
     delete router;
 }
  
  void 
  EventOutputPort::mapImpl (IndexMap* indices,
			    Index::Type type,
			    int maxBuffered)
  {
    OutputPort::mapImpl (indices, type, maxBuffered, (int) sizeof (Event));
  }

  void
  EventOutputPort::map (IndexMap* indices, Index::Type type)
  {
    assertOutput ();
    EventOutputPort::mapImpl (indices, type, MAX_BUFFERED_NO_VALUE);
  }

  
  void
  EventOutputPort::map (IndexMap* indices,
			Index::Type type,
			int maxBuffered)
  {
    assertOutput ();
    if (maxBuffered <= 0)
      {
	error ("EventOutputPort::map: maxBuffered should be a positive integer");
      }
    EventOutputPort::mapImpl (indices, type, maxBuffered);
  }

  
  Connector*
  EventOutputPort::makeConnector (ConnectorInfo connInfo)
  {
    Connector *conn;
    // we need to choose a right connector according to the communication type
    if (connInfo.communicationType () ==  ConnectorInfo::POINTTOPOINT)
      conn = new EventOutputConnector (connInfo,
				       indices_,
				       index_type_,
				       setup_->communicator (),
				       routingMap);
    else
      conn = new EventOutputCollectiveConnector
	(connInfo,
	 indices_,
	 index_type_,
	 setup_->communicator (),
	 router->directRouter ());

    return conn;
  }
  
  
  void
  EventOutputPort::insertEventImpl (double t, int id)
  {
    router->processEvent(t, id);
  }

  void
  EventOutputPort::insertEvent (double t, GlobalIndex id)
  {
    insertEventImpl(t, id);
  }

  
  void
  EventOutputPort::insertEvent (double t, LocalIndex id)
  {
    insertEventImpl(t, id);
  }

  void
  EventOutputPort::buildTable ()
  {
    routingMap->fillRouter (router);
    delete routingMap;
    router->buildTable ();
  }


  EventInputPort::EventInputPort (Setup* s, std::string id)
    : Port (s, id)
  {

  }
  

  void
  EventInputPort::map (IndexMap* indices,
		       EventHandlerGlobalIndex* handleEvent,
		       double accLatency)
  {
    assertInput ();
    int maxBuffered = MAX_BUFFERED_NO_VALUE;
    mapImpl (indices,
	     Index::GLOBAL,
	     EventHandlerPtr (handleEvent),
	     accLatency,
	     maxBuffered);
  }

  
  void
  EventInputPort::map (IndexMap* indices,
		       EventHandlerLocalIndex* handleEvent,
		       double accLatency)
  {
    assertInput ();
    int maxBuffered = MAX_BUFFERED_NO_VALUE;
    mapImpl (indices,
	     Index::LOCAL,
	     EventHandlerPtr (handleEvent),
	     accLatency,
	     maxBuffered);
  }

  
  void
  EventInputPort::map (IndexMap* indices,
		       EventHandlerGlobalIndex* handleEvent,
		       double accLatency,
		       int maxBuffered)
  {
    assertInput ();
    if (maxBuffered <= 0)
      {
	error ("EventInputPort::map: maxBuffered should be a positive integer");
      }
    mapImpl (indices,
	     Index::GLOBAL,
	     EventHandlerPtr (handleEvent),
	     accLatency,
	     maxBuffered);
  }

  
  void
  EventInputPort::map (IndexMap* indices,
		       EventHandlerLocalIndex* handleEvent,
		       double accLatency,
		       int maxBuffered)
  {
    assertInput ();
    if (maxBuffered <= 0)
      {
	error ("EventInputPort::map: maxBuffered should be a positive integer");
      }
    mapImpl (indices,
	     Index::LOCAL,
	     EventHandlerPtr (handleEvent),
	     accLatency,
	     maxBuffered);
  }

  
  void
  EventInputPort::mapImpl (IndexMap* indices,
			   Index::Type type,
			   EventHandlerPtr handleEvent,
			   double accLatency,
			   int maxBuffered)
  {
    type_ = type;
    handleEvent_ = handleEvent;
    InputPort::mapImpl (indices,
				      type,
				      accLatency,
				      maxBuffered,
				      false);
  }

  
  Connector*
  EventInputPort::makeConnector (ConnectorInfo connInfo)
  {
	  // we need to choose a right connector according to the communication type
	  Connector *conn;
	  if(connInfo.communicationType() ==  ConnectorInfo::POINTTOPOINT)
		  conn =   new EventInputConnector (connInfo,
		  			  indices_,
		  			  index_type_,
		  			  setup_->communicator (),
		  			  handleEvent_);
	  else
		  conn = new EventInputCollectiveConnector(connInfo,
	  	 			indices_,
		  			index_type_,
	  	 			setup_->communicator (),
	  	 			handleEvent_);

	 return conn;
  }

  
  EventHandlerGlobalIndexProxy*
  EventInputPort::allocEventHandlerGlobalIndexProxy (void (*eh) (double, int))
  {
    cEventHandlerGlobalIndex = EventHandlerGlobalIndexProxy (eh);
    return &cEventHandlerGlobalIndex;
  }

  
  EventHandlerLocalIndexProxy*
  EventInputPort::allocEventHandlerLocalIndexProxy (void (*eh) (double, int))
  {
    cEventHandlerLocalIndex = EventHandlerLocalIndexProxy (eh);
    return &cEventHandlerLocalIndex;
  }

  /********************************************************************
   *
   * Message Ports
   *
   ********************************************************************/

  MessagePort::MessagePort (Setup* s)
    : rank_ (mpi_get_rank (s->communicator ()))
  {
  }
  
  
  MessageOutputPort::MessageOutputPort (Setup* s, std::string id)
    : Port (s, id), MessagePort (s)
  {
  }

  
  void
  MessageOutputPort::map ()
  {
    assertOutput ();
    int maxBuffered = MAX_BUFFERED_NO_VALUE;
    mapImpl (maxBuffered);
  }

  
  void
  MessageOutputPort::map (int maxBuffered)
  {
    assertOutput ();
    if (maxBuffered <= 0)
      {
	error ("MessageOutputPort::map: maxBuffered should be a positive integer");
      }
    mapImpl (maxBuffered);
  }

  
  void
  MessageOutputPort::mapImpl (int maxBuffered)
  {
    // Identify ourselves
    LinearIndex indices (rank_, 1);
    OutputPort::mapImpl (&indices,
				       Index::GLOBAL,
				       maxBuffered,
				       1);
  }
  
  
  Connector*
  MessageOutputPort::makeConnector (ConnectorInfo connInfo)
  {
    return new MessageOutputConnector (connInfo,
				       indices_,
				       index_type_,
				       setup_->communicator (),
				       buffers);
  }
  
  
  void
  MessageOutputPort::insertMessage (double t, void* msg, size_t size)
  {
    // One output buffer per OutputConnector (since different
    // connectors may need to send at different times)
    for (std::vector<FIBO*>::iterator b = buffers.begin ();
	 b != buffers.end ();
	 ++b)
      {
	MessageHeader header (t, size);
	(*b)->insert (header.data (), sizeof (MessageHeader));
	(*b)->insert (msg, size);
      }
  }

  
  MessageInputPort::MessageInputPort (Setup* s, std::string id)
    : Port (s, id), MessagePort (s)
  {
  }

  
  void
  MessageInputPort::map (MessageHandler* handleMessage,
			 double accLatency)
  {
    assertInput ();
    int maxBuffered = MAX_BUFFERED_NO_VALUE;
    mapImpl (handleMessage,
	     accLatency,
	     maxBuffered);
  }

  
  void
  MessageInputPort::map (int maxBuffered)
  {
    assertInput ();
    if (maxBuffered <= 0)
      {
	error ("MessageInputPort::map: maxBuffered should be a positive integer");
      }
    mapImpl (0,
	     0.0,
	     maxBuffered);
  }

  
  void
  MessageInputPort::map (double accLatency,
			 int maxBuffered)
  {
    assertInput ();
    if (maxBuffered <= 0)
      {
	error ("MessageInputPort::map: maxBuffered should be a positive integer");
      }
    mapImpl (0,
	     accLatency,
	     maxBuffered);
  }

  
  void
  MessageInputPort::map (MessageHandler* handleMessage,
			 int maxBuffered)
  {
    assertInput ();
    if (maxBuffered <= 0)
      {
	error ("MessageInputPort::map: maxBuffered should be a positive integer");
      }
    mapImpl (handleMessage,
	     0.0,
	     maxBuffered);
  }

  
  void
  MessageInputPort::map (MessageHandler* handleMessage,
			 double accLatency,
			 int maxBuffered)
  {
    assertInput ();
    if (maxBuffered <= 0)
      {
	error ("MessageInputPort::map: maxBuffered should be a positive integer");
      }
    mapImpl (handleMessage,
	     accLatency,
	     maxBuffered);
  }

  
  void
  MessageInputPort::mapImpl (MessageHandler* handleMessage,
			     double accLatency,
			     int maxBuffered)
  {
    handleMessage_ = handleMessage;
    // Receive from everybody
    LinearIndex indices (0, handleMessage ? Index::WILDCARD_MAX : 0);
    InputPort::mapImpl (&indices,
				      Index::GLOBAL,
				      accLatency,
				      maxBuffered,
				      false);
  }

  
  Connector*
  MessageInputPort::makeConnector (ConnectorInfo connInfo)
  {
    return new MessageInputConnector (connInfo,
				      indices_,
		  			  index_type_,
				      handleMessage_,
				      setup_->communicator ());
  }

  
  MessageHandlerProxy*
  MessageInputPort::allocMessageHandlerProxy (void (*mh) (double,
								     void*,
								     size_t))
  {
    cMessageHandler = MessageHandlerProxy (mh);
    return &cMessageHandler;
  }

  
}
#endif
