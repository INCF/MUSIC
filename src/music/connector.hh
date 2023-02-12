/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007, 2008, 2009, 2012, 2022 INCF
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

#ifndef MUSIC_CONNECTOR_HH

#include "music/music-config.hh"

#if MUSIC_USE_MPI

#include <vector>
#include <string>

//#include <music/synchronizer.hh>
#include <music/FIBO.hh>
#include <music/event.hh>
#include <music/spatial.hh>
#include <music/connectivity.hh>
#include <music/sampler.hh>
#include <music/collector.hh>
#include <music/distributor.hh>
#include <music/event_routing_map.hh>
#include <music/clock.hh>
#include <music/subconnector.hh>

namespace MUSIC {
  /* remedius
   * New type of Connector was introduced: CollectiveConnector.
   * So that Connector class hierarchy undergoes the following changes:
   * the common functionality for OutputConnector and InputConnector classes was moved to the base Connector class.
   */
  // The Connector is responsible for one side of the communication
  // between the ports of a port pair.  An output port can have
  // multiple connectors while an input port only has one.  The method
  // connector::connect () creates one subconnector for each MPI
  // process we will communicate with on the remote side.

  /* remedius
   */

  class MultiBuffer;

  class Connector
  {
  private:
    static std::map<unsigned int, Connector*> flagMap_;
    static unsigned int nextFlag_;
    static unsigned int nextProxyFlag_;
  protected:
    ConnectorInfo info;
    IndexMap* indices_;
    Index::Type type_;
    MPI_Comm comm;
    MPI_Comm intercomm;
    std::vector<Subconnector*> rsubconn;
    ClockState latency_;
    // interpolate rather than picking value closest in time
    bool interpolate_;
    int width_;
    int maxLocalWidth_;
    unsigned int idFlag_;
    bool finalized_;

    Connector () { };
    Connector (ConnectorInfo info_,
	       IndexMap* indices,
	       Index::Type type,
	       MPI_Comm c);
    Connector (ConnectorInfo info_,
	       IndexMap* indices,
	       Index::Type type,
	       MPI_Comm c,
	       MPI_Comm ic);

    static unsigned int makeFlag (Connector* connector)
    {
      unsigned int flag = nextFlag_;
      nextFlag_ <<= 1;

      flagMap_[flag] = connector;

      return flag;
    }

    static unsigned int makeProxyFlag ()
    {
      unsigned int flag = nextProxyFlag_;
      nextProxyFlag_ <<= 1;
      return flag;
    }

  public:

    static Connector* connectorFromIdFlag (unsigned int flag)
    {
      return flagMap_[flag];
    }

    static unsigned int idRange () { return nextFlag_; }

    static unsigned int proxyIdRange () { return nextProxyFlag_; }

    void report ()
    {
      for (std::vector<Subconnector*>::iterator subconnector = rsubconn.begin ();
	   subconnector != rsubconn.end ();
	   ++subconnector)
	(*subconnector)->report ();
    }
    virtual ~Connector ()
    {
      for (std::vector<Subconnector*>::iterator subconnector = rsubconn.begin ();
	   subconnector != rsubconn.end ();
	   ++subconnector)
	delete *subconnector;
    }
    virtual void specialize (Clock& localTime) { }

    unsigned int idFlag () const { return idFlag_; }
    bool needsMultiCommunication () const { return idFlag (); }
    virtual bool isProxy () const { return false; }
    virtual bool isInput () const { return false; }
    std::string receiverAppName () const { return info.receiverAppName (); }
    std::string receiverPortName () const { return info.receiverPortName (); }
    int receiverPortCode () const { return info.receiverPortCode (); }
    int remoteLeader () const { return info.remoteLeader (); }
    int remoteNProcs () const { return info.nProcesses (); }
    int width(){return width_;}
    int maxLocalWidth () { return maxLocalWidth_; }
    bool isLeader ();
    virtual void setInterpolate(bool val){interpolate_ =  val;}
    virtual void setLatency( ClockState latency){ latency_ = latency;}
    // virtual Synchronizer* synchronizer () = 0;
    virtual void createIntercomm ();
    virtual void freeIntercomm ();

    /* remedius
     * since there can be different types of Subconnectors,
     * Instead of adding new type of Subconnector,
     * function parameters' were changed to one parameter without specifying the type of Subconnector and
     * the common functionality for InputConnector::spatialNegotiation and OutputConnector::spatialNegotiation
     * was coded in this base Connector::spatialNegotiation() method.
     * Returns SubconnectorType.
     */
    virtual void spatialNegotiation ();
    /* remedius
     *  sorts its vector of subconnectors according to its world rank.
     */
    virtual void initialize ();

    /* remedius
     * finalize method iterates its subconnectors and calls accordring flush method
     */
    virtual void finalize ();

    virtual bool isFinalized () { return finalized_; }

    /* remedius
     * tick method iterates its subconnectors and call according function to perform communication;
     * requestCommunication method argument was removed since there is no need anymore to check for next communication.
     * Communication is scheduled by separate Scheduler objects that Runtime object contains.
     */
    virtual void tick ();
    /* remedius
     * In order to carry out the base functionality to the spatialNegotiation() method
     * the two following common virtual methods were added so that
     * all successors of this class had to override abstract makeSubconnector method
     * instead of implementing it's own make^SpecificType^Subconnector() method.
     */
  protected:
    virtual void addRoutingInterval (IndexInterval i, Subconnector* s){};
    virtual Subconnector* makeSubconnector (int remoteRank){return NULL;};

    virtual void spatialNegotiation (SpatialNegotiator *spatialNegotiator_);
    virtual SpatialNegotiator *createSpatialNegotiator(){return NULL;};
  };


  class ProxyConnector : virtual public Connector {
    int senderNode_;
    int senderLeader_;
    int senderNProcs_;
    int receiverNode_;
    int receiverLeader_;
    int receiverNProcs_;
    int currentNode_;
  public:
    ProxyConnector (int senderNode, int senderLeader, int senderNProcs,
		    int receiverNode, int receiverLeader, int receiverNProcs)
      : senderLeader_ (senderLeader), senderNProcs_ (senderNProcs),
	receiverLeader_ (receiverLeader), receiverNProcs_ (receiverNProcs)
    { idFlag_ = makeProxyFlag (); }
    ProxyConnector (){idFlag_ =0;}
    bool isProxy () const { return true; }
    void tick () { }
    void setNode (int nodeId)
    {
      currentNode_ = nodeId;
      //*fixme*
      if (isInput ())
	info.setRemoteLeader (senderLeader ());
      else
	info.setRemoteLeader (receiverLeader ());
    }
    using Connector::isInput;
    bool isInput () { return receiverNode_ == currentNode_; }
    int senderLeader () const { return senderLeader_; }
    int senderNProcs () const { return senderNProcs_; }
    int receiverLeader () const { return receiverLeader_; }
    int receiverNProcs () const { return receiverNProcs_; }
  };


  class PostCommunicationConnector : virtual public Connector {
  public:
    virtual void postCommunication () = 0;
  };
  class PreCommunicationConnector : virtual public Connector {
  public:
    virtual void preCommunication () = 0;
  };
  class OutputConnector : virtual public Connector {
  public:
#if MUSIC_ISENDWAITALL
   virtual void tick ();
#endif // MUSIC_ISENDWAITALL
  protected:
    OutputConnector(){};
    SpatialNegotiator *createSpatialNegotiator();
  };

  class InputConnector : virtual public Connector {
/*
  public:
#ifdef MUSIC_ANYSOURCE
	  virtual void tick ();
#endif // MUSIC_ANYSOURCE
*/
  protected:
    InputConnector(){};
    bool isInput () const { return true; }
    SpatialNegotiator *createSpatialNegotiator();
  };


  class ContConnector : virtual public Connector {
  protected:
    Sampler& sampler_;
    MPI_Datatype  data_type_;

    Clock *localTime_;
    Clock remoteTime_;

  public:
    ContConnector (Sampler& sampler, MPI_Datatype type)
      : sampler_ (sampler),  data_type_ (type),localTime_(NULL) { }
    ClockState remoteTickInterval (ClockState tickInterval);
    MPI_Datatype getDataType(){return  data_type_;}
    virtual void initialize ();
  protected:
    virtual void initialCommunication()=0;
    friend class SpecializedContConnector;
  };

  class ContOutputConnector : public ContConnector, public OutputConnector, public PreCommunicationConnector  {
  protected:
    Distributor distributor_;
    PreCommunicationConnector* connector; //specialized connector
    ContOutputConnector(Sampler& sampler, MPI_Datatype type);
  public:
    ContOutputConnector (ConnectorInfo connInfo,
			 IndexMap* indices,
			 Index::Type type,
			 MPI_Comm comm,
			 Sampler& sampler,
			 MPI_Datatype data_type);
    ~ContOutputConnector();
    //MUSIC_ISENDWAITALL is not supported temporarily for cont. ports
#if MUSIC_ISENDWAITALL
    void tick (){Connector::tick();};
#endif
    void specialize (Clock& localTime);
    void preCommunication () {connector->preCommunication(); }
    void initialize (){connector->initialize();ContConnector::initialize();};
  protected:
    void addRoutingInterval (IndexInterval i, Subconnector* osubconn);
    Subconnector* makeSubconnector (int remoteRank);
    void initialCommunication();
    friend class SpecializedContOuputConnector;
  };

  class ContInputConnector : public ContConnector,  public InputConnector,  public PostCommunicationConnector {

  protected:
    Collector collector_;
    double delay_;
    PostCommunicationConnector* connector;
    bool divisibleDelay (Clock& localTime);
    ContInputConnector(Sampler& sampler,MPI_Datatype type,	double delay);
  public:
    ContInputConnector (ConnectorInfo connInfo,
			IndexMap* indices,
			Index::Type type,
			MPI_Comm comm,
			Sampler& sampler,
			MPI_Datatype data_type,
			double delay);
    ~ContInputConnector();
    void specialize (Clock& localTime);
    void postCommunication () {connector->postCommunication(); }
    void initialize (){connector->initialize();ContConnector::initialize();};
    //MUSIC_ANYSOURCE is not supported temporarily for cont. ports
//#ifdef MUSIC_ANYSOURCE
//  virtual void tick {Connector::tick();};
//#endif // MUSIC_ANYSOURCE
  protected:
    void addRoutingInterval (IndexInterval i, Subconnector* isubconn);
    void initialCommunication();
    Subconnector* makeSubconnector (int remoteRank);
  private:
    int initialBufferedTicks();
    friend class SpecializedContInputConnector;
  };


  class EventOutputConnector : public OutputConnector{
  private:
    EventRoutingMap<FIBO*>* routingMap_;
  public:
    EventOutputConnector (ConnectorInfo connInfo,
			  IndexMap* indices,
			  Index::Type type,
			  MPI_Comm comm,
			  EventRoutingMap<FIBO*>* routingMap): Connector(connInfo, indices, type ,comm),
							       routingMap_(routingMap){
    };
    ~EventOutputConnector(){}

  protected:
    EventOutputConnector():routingMap_(NULL){};
    EventOutputConnector(EventRoutingMap<FIBO*>* routingMap): routingMap_(routingMap){};
    Subconnector* makeSubconnector (int remoteRank);
    void addRoutingInterval (IndexInterval i, Subconnector* subconn);
  };
  class EventInputConnector : public InputConnector {
  protected:
    //	EventRoutingMap<EventHandlerGlobalIndex*>* routingMap_;
    EventHandlerPtr handleEvent_;
    std::map<int,EventInputSubconnectorGlobal*> rRank2Subconnector;
    int flushes;
  public:

    EventInputConnector (ConnectorInfo connInfo,
			 IndexMap* indices,
			 Index::Type type,
			 MPI_Comm comm,
			 EventHandlerPtr handleEvent):Connector(connInfo, indices, type ,comm),
						      handleEvent_(handleEvent),flushes(0){};
    ~EventInputConnector(){}
    //probably should be moved to InputConnector later
#if MUSIC_ANYSOURCE
  void tick ();
#endif // MUSIC_ANYSOURCE
  protected:
    EventInputConnector(){};
    EventInputConnector (EventHandlerPtr handleEvent): handleEvent_(handleEvent){};
    Subconnector* makeSubconnector (int remoteRank);
    /* remedius
     * original version of the following function does nothing,
     * however this function could perform inserting interval to the routingMap_ as well,
     * that is currently done in CollectiveConnector::addRoutingInterval.
     * This could give an opportunity to have different event handlers for different ranges of indexes as in collective algorithm.
     * Do we need this? If not, then it's better to leave as it is.
     */
    void addRoutingInterval (IndexInterval i, Subconnector* subconn);
  };


  class MessageOutputConnector : public OutputConnector,
				 public PostCommunicationConnector {
  private:
    //   OutputSynchronizer synch;
    FIBO buffer; // MessageOutputConnectors have only one output buffer
    bool bufferAdded;
    std::vector<FIBO*>& buffers_; // pointer to the port vector of
				  // connector buffers
    void send ();
  public:
    MessageOutputConnector (ConnectorInfo connInfo,
			    IndexMap* indices,
			    Index::Type type,
			    MPI_Comm comm,
			    std::vector<FIBO*>& buffers);

    void postCommunication ();
    //MUSIC_ISENDWAITALL is not supported temporarily for message ports
#if MUSIC_ISENDWAITALL
    void tick (){Connector::tick();};
#endif
  protected:
    void addRoutingInterval (IndexInterval i, Subconnector* subconn);
    Subconnector* makeSubconnector (int remoteRank);
    SpatialNegotiator *createSpatialNegotiator() {return OutputConnector::createSpatialNegotiator();};
  };

  class MessageInputConnector : public InputConnector {

  private:
    MessageHandler* handleMessage_;
  public:
    MessageInputConnector (ConnectorInfo connInfo,
			   IndexMap* indices,
			   Index::Type type,
			   MessageHandler* handleMessage,
			   MPI_Comm comm);
    //MUSIC_ANYSOURCE is not supported temporarily for message ports
/*
#ifdef MUSIC_ANYSOURCE
  virtual void tick {Connector::tick();};
#endif // MUSIC_ANYSOURCE
*/
  protected:
    Subconnector* makeSubconnector (int remoteRank);
  };
  /* remedius
   * New class hierarchy CollectiveConnector was introduced in order to create CollectiveSubconnector object.
   * The difference between InputConnector, OutputConnector instances and  CollectiveConnector is
   * the latest creates one CollectiveSubconnector responsible for the collective communication
   * among all ranks on the output and input sides, while InputConnector and OutputConnector
   * instances are responsible for creating as much Subconnectors as necessary for each point2point connectivity
   * for output and input sides.
   */

  class CollectiveConnector: virtual public Connector {
    bool high_;
  protected:
    MPI_Comm intracomm_;
    Subconnector* subconnector_;
  public:
    CollectiveConnector (bool high);
    using Connector::makeSubconnector;
    virtual Subconnector* makeSubconnector (void *param) = 0;
    void createIntercomm ();
    void freeIntercomm ();
    Subconnector* subconnector () { return subconnector_; }
  };

  class ContCollectiveConnector: public CollectiveConnector {
    MPI_Datatype data_type;
  protected:
    ContCollectiveConnector( MPI_Datatype type, bool high);
    using Connector::makeSubconnector;
    Subconnector* makeSubconnector (void *param);
  };

  class EventCollectiveConnector: public CollectiveConnector {
  protected:
    EventRouter *router_; //is used for processing received information
    EventCollectiveConnector( bool high);
  public:
    void createIntercomm () { Connector::createIntercomm (); }
    void freeIntercomm () { Connector::freeIntercomm (); }
  };

  class EventInputCollectiveConnector:  public EventInputConnector, public EventCollectiveConnector {
    EventRoutingMap<EventHandlerPtr*>* routingMap_input; //is used to fill the information for the receiver
  public:
    EventInputCollectiveConnector(ConnectorInfo connInfo,
				  IndexMap* indices,
				  Index::Type type,
				  MPI_Comm comm,
				  EventHandlerPtr handleEvent);
    ~EventInputCollectiveConnector();
#if MUSIC_ANYSOURCE
  void tick() { Connector::tick();};
#endif // MUSIC_ANYSOURCE
  protected:
    using Connector::spatialNegotiation;
    using EventInputConnector::makeSubconnector;
    void spatialNegotiation ( SpatialNegotiator* spatialNegotiator_);
    void addRoutingInterval(IndexInterval i, Subconnector* subconn);
    Subconnector* makeSubconnector (void *param);
    void finalize () { };
    bool isFinalized () { return subconnector ()->isFlushed (); }
  };

  class DirectRouter;

  class EventOutputCollectiveConnector:  public EventOutputConnector,public EventCollectiveConnector{
    DirectRouter* directRouter_;
  public:
    EventOutputCollectiveConnector(ConnectorInfo connInfo,
				   IndexMap* indices,
				   Index::Type type,
				   MPI_Comm comm,
				   DirectRouter* router);
    ~EventOutputCollectiveConnector();
#if MUSIC_ISENDWAITALL
    void tick (){Connector::tick();};
#endif //MUSIC_ISENDWAITALL
  protected:
    using Connector::spatialNegotiation;
    using EventOutputConnector::makeSubconnector;
    void spatialNegotiation ( SpatialNegotiator* spatialNegotiator_);
    Subconnector* makeSubconnector (void *param);
  };

  class ContInputCollectiveConnector: public ContInputConnector, public ContCollectiveConnector
  {
  public:
    ContInputCollectiveConnector(ConnectorInfo connInfo,
				 IndexMap* indices,
				 Index::Type type,
				 MPI_Comm comm,
				 Sampler& sampler,
				 MPI_Datatype data_type,
				 double delay);
#if MUSIC_ANYSOURCE
  virtual void tick() {Connector::tick();};
#endif // MUSIC_ANYSOURCE
  protected:
    using Connector::spatialNegotiation;
    void spatialNegotiation ( SpatialNegotiator* spatialNegotiator_);
  private:
    void receiveRemoteCommRankID(std::map<int,int> &remoteToCollectiveRankMap);
  };

  class ContOutputCollectiveConnector:  public ContOutputConnector, public ContCollectiveConnector
  {
  public:
    ContOutputCollectiveConnector(ConnectorInfo connInfo,
				  IndexMap* indices,
				  Index::Type type,
				  MPI_Comm comm,
				  Sampler& sampler,
				  MPI_Datatype data_type);
#if MUSIC_ISENDWAITALL
    void tick (){Connector::tick();};
#endif
  protected:
    using Connector::spatialNegotiation;
    void spatialNegotiation ( SpatialNegotiator* spatialNegotiator_);
  private:
    void sendLocalCommRankID();
  };
  class SpecializedContConnector{
  protected:
    Sampler& sampler_;
    Clock *localTime_;
    Clock &remoteTime_;
    ClockState &latency;
    bool &interp;
    SpecializedContConnector(ContConnector *conn):
    sampler_ ( conn->sampler_),
    localTime_(conn->localTime_),
    remoteTime_(conn->remoteTime_),
    latency( conn->latency_),
    interp( conn->interpolate_){};
  };
  class SpecializedContOuputConnector:public SpecializedContConnector{
  protected:
    Distributor &distributor_;
    SpecializedContOuputConnector( ContOutputConnector *conn):
    SpecializedContConnector(conn),
    distributor_(conn->distributor_){ }
  };
  class SpecializedContInputConnector:public SpecializedContConnector{
  protected:
    Collector &collector_;
    SpecializedContInputConnector( ContInputConnector *conn):
    SpecializedContConnector(conn),
    collector_(conn->collector_){}
  };
  class PlainContOutputConnector : public PreCommunicationConnector, public SpecializedContOuputConnector {
  protected:
    PlainContOutputConnector ( ContOutputConnector *conn):SpecializedContOuputConnector(conn){};
  public:
    void initialize ();
    void preCommunication();
  private:
    bool sample();
    friend class ContOutputConnector;

  };

  class InterpolatingContOutputConnector :  public PreCommunicationConnector, public SpecializedContOuputConnector
  {
  protected:
    InterpolatingContOutputConnector (ContOutputConnector *conn):SpecializedContOuputConnector(conn){};
  public:
    void initialize ();
    void preCommunication();
  private:
    bool interpolate();
    bool sample();
    double interpolationCoefficient();
    friend class ContOutputConnector;

  };
  class PlainContInputConnector : public PostCommunicationConnector, public SpecializedContInputConnector {
  protected:
    PlainContInputConnector (ContInputConnector *conn):SpecializedContInputConnector(conn){};
  public:
    void initialize ();
    void postCommunication ();
    friend class ContInputConnector;
  };

  class InterpolatingContInputConnector : public PostCommunicationConnector, public SpecializedContInputConnector  {
    bool first_;
  protected:
    InterpolatingContInputConnector (ContInputConnector *conn):SpecializedContInputConnector(conn){};
  public:
    void initialize ();
    void postCommunication ();
  private:
    bool sample();
    double interpolationCoefficient();
    friend class ContInputConnector;
  };
}
#endif
#define MUSIC_CONNECTOR_HH
#endif
