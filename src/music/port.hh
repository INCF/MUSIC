/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007, 2008, 2009 INCF
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

#ifndef MUSIC_PORT_HH
#include "music/music-config.hh"
#if MUSIC_USE_MPI
#include <mpi.h>

#include <string>

#include <music/data_map.hh>
#include <music/index_map.hh>
#include <music/event.hh>
#include <music/message.hh>
#include <music/connector.hh>
#include <music/connection.hh>
#include <music/sampler.hh>
#include <music/event_routing_map.hh>
#include <music/connectivity.hh>
#include <music/spatial.hh>

namespace MUSIC {

  class Application;
  class PortConnectivityManager;

  using Connections = std::vector<Connection*>;
/* remedius
 * 1. Since one more type of Connector was added - CollectiveConnector,
 * to avoid branching of different Connector return types,
 * all makeOutputConnector, makeInputConnector methods were replaced with
 * Connector* makeConnector method and placed to the base class Port.
 * 2. RedistributionPort by its definition didn't fit the current meaning
 * for Input/Output ports anymore, hence it was removed.
 * All the functionality from Input/OutputRedistributionPort
 * has been moved to Input/OutputPort respectively.
 */
  class Port {
  public:
    /* Port () { } */
    explicit Port (const Application& app, std::string identifier);

    bool isConnected ();
    bool hasWidth ();
    int width ();
    void checkIndexMap (IndexMap* indexMap);
	std::string name ();

  protected:
    IndexMap* indices_;
    Index::Type index_type_ {Index::GLOBAL};
    std::string portName_;
    const Application& app_;
	Connections connections_;

    virtual void buildTable () {};
	virtual void reconnect () = 0;
	virtual Connections& getConnections ();
    virtual Connector* makeConnector (ConnectorInfo connInfo) = 0;
	const ConnectivityInfo& getConnectivityInfo () const;
	void removeConnections ();
    void assertOutput ();
    void assertInput ();

  public: // MDJ 2012-08-07 public for now---see comment in runtime.cc
    virtual ~Port();

  private:
    void checkConnected (std::string action);
    bool isMapped_;
    friend class Runtime;
	friend class PortConnectivityManager;
  };


  // A ticking port is a port which needs to be updated at every tick ()

  class TickingPort : public virtual Port {
  public:
	using Port::Port;
    virtual void tick () = 0;
  };


  class OutputPort :  public  virtual Port {
  public:
	  using Port::Port;
  protected:

	void reconnect () override;
    virtual void mapImpl (IndexMap* indices,
			  Index::Type type,
			  int maxBuffered,
			  int dataSize);
    friend class Implementer;
  };

  class InputPort :   public  virtual Port {
  public:
	  using Port::Port;
  protected:

	void reconnect () override;
    void mapImpl (IndexMap* indices,
		  Index::Type type,
		  double accLatency,
		  int maxBuffered,
		  bool interpolate);
    friend class Implementer;
  };

  class ContPort :  public virtual Port {
  public:
	using Port::Port;
  protected:
    Sampler sampler;
    MPI::Datatype type_;
  };

  class ContOutputPort : public ContPort,
			 public OutputPort,
			 public TickingPort {
    void mapImpl (DataMap* indices, int maxBuffered);
    Connector* makeConnector (ConnectorInfo connInfo);
    friend class Implementer;

  public:
    ContOutputPort (const Application& app, std::string id)
      : Port (app, id) { }
    void map (DataMap* dmap);
    void map (DataMap* dmap, int maxBuffered);
    void tick ();
  };

  class ContInputPort : public ContPort, public InputPort {
    double delay_;
    void mapImpl (DataMap* dmap,
		  double delay,
		  int maxBuffered,
		  bool interpolate);
    Connector* makeConnector (ConnectorInfo connInfo);
    friend class Implementer;

  public:
    ContInputPort (const Application& app, std::string id)
      : Port (app, id) { }
    void map (DataMap* dmap, double delay = 0.0, bool interpolate = true);
    void map (DataMap* dmap, int maxBuffered, bool interpolate = true);
    void map (DataMap* dmap,
	      double delay,
	      int maxBuffered,
	      bool interpolate = true);
  };



  /* remedius
   * EventOutputPort constructor, makeOutputConnector() and buildTable() methods were hided from the user.
   * In order to give the Setup class access to EventOutputPort constructor,
   * Setup class was declared as a friend class to EventOutputPort.
   */
  class EventOutputPort :  public OutputPort {
	EventRouter *router;
    EventRoutingMap<FIBO*>* routingMap;
  public:
    void map (IndexMap* indices, Index::Type type);
    void map (IndexMap* indices, Index::Type type, int maxBuffered);
    void insertEvent (double t, GlobalIndex id);
    void insertEvent (double t, LocalIndex id);
    EventOutputPort (const Application& app, std::string id);

    void mapImpl (IndexMap* indices,
		  Index::Type type,
		  int maxBuffered);
	void reconnect () override;
    void insertEventImpl (double t, int id);

  public: // MDJ 2012-08-07 public for now---see comment in runtime.cc
    ~EventOutputPort();

  private:
    Connector* makeConnector (ConnectorInfo connInfo);
    void buildTable ();
    friend class Application;
    friend class Implementer;
  };
/* remedius
 * EventInputPort constructor was hided from the user.
 * In order to give the Setup class access to EventInputPort constructor,
 * Setup class was declared as a friend class to EventInputPort.
 * routingMap field and  buildTable() method were added to EventInputPort class since
 * the routing of the data can also happen on the input side (collective algorithm).
 */
  class EventInputPort : public InputPort {
  private:
	Index::Type type_;
    EventHandlerPtr handleEvent_;
   // EventRoutingMap<EventHandlerGlobalIndex*>* routingMap;
  public:
    void map (IndexMap* indices,
	      EventHandlerGlobalIndex* handleEvent,
	      double accLatency = 0.0);
    void map (IndexMap* indices,
	      EventHandlerLocalIndex* handleEvent,
	      double accLatency = 0.0);
    void map (IndexMap* indices,
	      EventHandlerGlobalIndex* handleEvent,
	      double accLatency,
	      int maxBuffered);
    void map (IndexMap* indices,
	      EventHandlerLocalIndex* handleEvent,
	      double accLatency,
	      int maxBuffered);
    EventInputPort (const Application& app, std::string id);
  protected:
    void mapImpl (IndexMap* indices,
		  Index::Type type,
		  EventHandlerPtr handleEvent,
		  double accLatency,
		  int maxBuffered);


    Connector* makeConnector (ConnectorInfo connInfo);
    // Facilities to support the C interface
  public:
    EventHandlerGlobalIndexProxy*
    allocEventHandlerGlobalIndexProxy (void (*) (double, int));
    EventHandlerLocalIndexProxy*
    allocEventHandlerLocalIndexProxy (void (*) (double, int));
  private:
    EventHandlerGlobalIndexProxy cEventHandlerGlobalIndex;
    EventHandlerLocalIndexProxy cEventHandlerLocalIndex;

    friend class Application;
    friend class Implementer;
  };


  class MessagePort : public virtual Port {
  protected:
    int rank_;
  public:
    MessagePort (const Application& app);
  };

  class MessageOutputPort : public MessagePort,
			    public OutputPort {
    std::vector<FIBO*> buffers; // one buffer per MessageOutputConnector
  public:
    MessageOutputPort (const Application& app, std::string id);
    void map ();
    void map (int maxBuffered);
    void insertMessage (double t, void* msg, size_t size);
  protected:
    void mapImpl (int maxBuffered);
    Connector* makeConnector (ConnectorInfo connInfo);
    friend class Implementer;
  };

  class MessageInputPort : public MessagePort,
			   public InputPort {
    MessageHandler* handleMessage_;
  public:
    MessageInputPort (const Application& app, std::string id);
    void map (MessageHandler* handler = 0, double accLatency = 0.0);
    void map (int maxBuffered);
    void map (double accLatency, int maxBuffered);
    void map (MessageHandler* handler, int maxBuffered);
    void map (MessageHandler* handler, double accLatency, int maxBuffered);
  protected:
    void mapImpl (MessageHandler* handleEvent,
		  double accLatency,
		  int maxBuffered);
    Connector* makeConnector (ConnectorInfo connInfo);
    friend class Implementer;

  public:
    MessageHandlerProxy*
    allocMessageHandlerProxy (void (*) (double, void*, size_t));
  private:
    MessageHandlerProxy cMessageHandler;
  };


}
#endif
#define MUSIC_PORT_HH
#endif
