/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007, 2008, 2009, 2022 INCF
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

#ifndef MUSIC_SUBCONNECTOR_HH
#include "music/music-config.hh"
#if MUSIC_USE_MPI
#include <mpi.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
//#include <music/synchronizer.hh>
#include <music/FIBO.hh>
#include <music/BIFO.hh>
#include <music/event.hh>
#include <music/message.hh>
#include <music/mpi_utils.hh>

namespace MUSIC {

  // NOTE: Must be divisible by the size of the datatype of the data
  // maps passed to cont ports
  const int SPIKE_BUFFER_MAX = 10000 * sizeof (Event);
  const int CONT_BUFFER_MAX = SPIKE_BUFFER_MAX; //5000 of double values per input connector
  const int MESSAGE_BUFFER_MAX = 10000;

  class EventRouter;

  // The subconnector is responsible for the local side of the
  // communication between two MPI processes, one for each port of a
  // port pair.  It is created in connector::connect ().
  
  class Subconnector {
  private:
  protected:
    //Synchronizer* synch;
    MPI_Datatype type_;
    MPI_Comm intercomm;
    int remoteRank_;		// rank in inter-communicatir
    int remoteWorldRank_;	// rank in COMM_WORLD
    int receiverRank_;
    int receiverPortCode_;
    bool flushed;

  public:
    //*fixme* should not create subconnectors with uninitialized members
    Subconnector () : type_ (MPI_BYTE), flushed (false) {};
    Subconnector (MPI_Datatype type):type_ (type), flushed(false) { }
    Subconnector (MPI_Datatype type,
		  MPI_Comm intercomm,
		  int remoteLeader,
		  int remoteRank,
		  int receiverRank,
		  int receiverPortCode);
    virtual ~Subconnector ();
    virtual void report () { }
    virtual void initialCommunication (double param) { }
    virtual void maybeCommunicate () = 0;
    virtual void maybeCommunicate (std::vector<MPI_Request> &) {};
    virtual void flush (bool& dataStillFlowing) = 0;
    bool isFlushed () { return flushed; }
    int localRank () const { return mpi_get_rank (intercomm); }
    int remoteRank () const { return remoteRank_; }
    //*fixme* are the following still needed or can they be removed?
    int remoteWorldRank () const { return remoteWorldRank_; }
    int receiverRank () const { return receiverRank_; }
    int receiverPortCode () const { return receiverPortCode_; }
  };
  
  class OutputSubconnector : virtual public Subconnector {
  protected:
    OutputSubconnector () {};
  public:
    virtual FIBO* outputBuffer () { return NULL; }
    // the following is part of the MultiConnector protocol
    virtual void nextBlock () { }
    virtual unsigned int dataSize () { return 0; }
    virtual void setOutputBuffer (void* buffer, unsigned int size) { }
    virtual void fillOutputBuffer () { }
  };
  
  class BufferingOutputSubconnector : virtual public OutputSubconnector {
  protected:
    FIBO buffer_;
  public:
    BufferingOutputSubconnector (int elementSize);
    FIBO* outputBuffer () { return &buffer_; }
    void report ()
    {
      std::cout << "Subconnector buffer size: " << buffer_.size () << std::endl; 
    }
  };

  class InputSubconnector : virtual public Subconnector {
  protected:
    InputSubconnector (){};
  public:
    virtual BIFO* inputBuffer () { return NULL; }
    // the following is part of the MultiConnector protocol
    virtual void processData (void* data, unsigned int size) { }
  };

  class ContOutputSubconnector : public BufferingOutputSubconnector{
  protected:
	  ContOutputSubconnector():BufferingOutputSubconnector (0){};
  public:
    ContOutputSubconnector (//Synchronizer* synch,
			    MPI_Comm intercomm,
			    int remoteLeader,
			    int remoteRank,
			    int receiverPortCode,
			    MPI_Datatype type);
    void initialCommunication (double param);
    using Subconnector::maybeCommunicate;
    void maybeCommunicate ();
    void send ();
    void flush (bool& dataStillFlowing);
  };
  
  class ContInputSubconnector : public InputSubconnector {
  protected:
    BIFO buffer_;
    ContInputSubconnector(){};
  public:
    ContInputSubconnector (//Synchronizer* synch,
			   MPI_Comm intercomm,
			   int remoteLeader,
			   int remoteRank,
			   int receiverRank,
			   int receiverPortCode,
			   MPI_Datatype type);
    BIFO* inputBuffer () { return &buffer_; }
    void initialCommunication (double initialBufferedTicks);
    using Subconnector::maybeCommunicate;
    void maybeCommunicate ();
    void receive ();
    void flush (bool& dataStillFlowing);
  };

  class EventSubconnector : virtual public Subconnector {
  protected:
    static const int FLUSH_MARK = -1;
  };
  


  class EventOutputSubconnector : public BufferingOutputSubconnector,
  				  public EventSubconnector {

  protected:
	  EventOutputSubconnector(): BufferingOutputSubconnector (sizeof (Event)){};
    public:

	  EventOutputSubconnector (//Synchronizer* synch,
  			     MPI_Comm intercomm,
  			     int remoteLeader,
  			     int remoteRank,
  			     int receiverPortCode);
      void maybeCommunicate ();
      void maybeCommunicate (std::vector<MPI_Request> &);

      void send ();
      void send (std::vector<MPI_Request> &);
      void flush (bool& dataStillFlowing);

    };
  class EventInputSubconnector : public InputSubconnector,
 				 public EventSubconnector {
   protected:
 	  EventInputSubconnector(){};
   public:
     EventInputSubconnector (//Synchronizer* synch,
 			    MPI_Comm intercomm,
 			    int remoteLeader,
 			    int remoteRank,
 			    int receiverRank,
 			    int receiverPortCode);
     using Subconnector::maybeCommunicate;
 	 void maybeCommunicate ();
     virtual void receive () {};
     virtual void flush (bool& dataStillFlowing);
   };

  class EventInputSubconnectorGlobal : public EventInputSubconnector {
    EventHandlerGlobalIndex* handleEvent;
  //  static EventHandlerGlobalIndexDummy dummyHandler;

  public:
    EventInputSubconnectorGlobal (//Synchronizer* synch,
				  MPI_Comm intercomm,
				  int remoteLeader,
				  int remoteRank,
				  int receiverRank,
				  int receiverPortCode,
				  EventHandlerGlobalIndex* eh);
    bool receive(char *data, int size);
    void receive ();
  //  void flush (bool& dataStillFlowing);
  };

  class EventInputSubconnectorLocal : public EventInputSubconnector {
    EventHandlerLocalIndex* handleEvent;
 //   static EventHandlerLocalIndexDummy dummyHandler;
  public:
    EventInputSubconnectorLocal (//Synchronizer* synch,
				 MPI_Comm intercomm,
				 int remoteLeader,
				 int remoteRank,
				 int receiverRank,
				 int receiverPortCode,
				 EventHandlerLocalIndex* eh);
    void receive ();
  //  void flush (bool& dataStillFlowing);
  };

  class MessageSubconnector : virtual public Subconnector {
  protected:
    static const int FLUSH_MARK = -1;
  };
  
  class MessageOutputSubconnector : public OutputSubconnector,
				  public MessageSubconnector {
    FIBO* buffer_;
  public:
    MessageOutputSubconnector (//Synchronizer* synch,
			       MPI_Comm intercomm,
			       int remoteLeader,
			       int remoteRank,
			       int receiverPortCode,
			       FIBO* buffer);
    using Subconnector::maybeCommunicate;
    void maybeCommunicate ();
    void send ();
    void flush (bool& dataStillFlowing);
  };
  
  class MessageInputSubconnector : public InputSubconnector,
				   public MessageSubconnector {
    MessageHandler* handleMessage;
  //  static MessageHandlerDummy dummyHandler;
  public:
    MessageInputSubconnector (//Synchronizer* synch,
			      MPI_Comm intercomm,
			      int remoteLeader,
			      int remoteRank,
			      int receiverRank,
			      int receiverPortCode,
			      MessageHandler* mh);
    using Subconnector::maybeCommunicate;
    void maybeCommunicate ();
    void receive ();
    void flush (bool& dataStillFlowing);
  };

  /* remedius
   * CollectiveSubconnector class is used for collective communication
   * based on the MPI_Allgather function.
   */
  class CollectiveSubconnector : public virtual Subconnector
  {
    int nProcesses, *ppBytes, *displ;
  protected:
    MPI_Comm intracomm_;

  protected:
    virtual ~CollectiveSubconnector ();
    CollectiveSubconnector (MPI_Comm intracomm);
    using Subconnector::maybeCommunicate;
    void maybeCommunicate ();
    int calcCommDataSize (int local_data_size);
    std::vector<char> getCommData(char *cur_buff, int size);
    const int* getDisplArr(){return displ;}
    const int* getSizeArr(){return ppBytes;}
    virtual void communicate () { };
    void allocAllgathervArrays ();
    void freeAllgathervArrays ();
  };


  class DirectRouter;
  

  class EventOutputCollectiveSubconnector
    : public OutputSubconnector,
      public EventSubconnector,
      public CollectiveSubconnector
  {
    DirectRouter* router_;
  public:
    EventOutputCollectiveSubconnector ()
      : CollectiveSubconnector (intracomm_)
    { }
    void setRouter (DirectRouter* router) { router_ = router; }

    // the following is part of the MultiConnector protocol
    void nextBlock () { }
    unsigned int dataSize ();
    void setOutputBuffer (void* buffer, unsigned int size);
    void fillOutputBuffer ();

    void flush (bool& dataStillFlowing) { }
    using Subconnector::maybeCommunicate;
    void maybeCommunicate () { }
  };


  class EventInputCollectiveSubconnector
    : public EventInputSubconnector, public CollectiveSubconnector
  {
  protected:
    EventRouter *router_;
  public:
    EventInputCollectiveSubconnector (EventRouter *router)
    : CollectiveSubconnector (intracomm_),
      router_(router)
    { }
    // the following is part of the MultiConnector protocol
    void processData (void* data, unsigned int size);
  private:
    using Subconnector::maybeCommunicate;
    void maybeCommunicate () { }
  };


    class ContCollectiveSubconnector
      : public ContInputSubconnector,
	public ContOutputSubconnector,
	public CollectiveSubconnector {
      std::multimap< int, Interval> intervals_; //the data we want
      std::map<int,int> blockSizePerRank_; //contains mapping of rank->size of the communication data
    										//(size of communication data/nBuffered per rank is constant).
      int width_; //port width in bytes
    public:
      ContCollectiveSubconnector (std::multimap<int, Interval> intervals,
				  int width,
				  MPI_Comm intracomm,
				  MPI_Datatype type)
	: Subconnector(type),
	  CollectiveSubconnector (intracomm),
	  intervals_ (intervals),
	  width_ (width * mpi_get_type_size (type))
      {
	allocAllgathervArrays ();
      }
      ~ContCollectiveSubconnector ()
      {
	void freeAllgathervArrays ();
      }
      void initialCommunication (double initialBufferedTicks);
      using Subconnector::maybeCommunicate;
      void maybeCommunicate ();
      void flush (bool& dataStillFlowing);
    private:
      void communicate();
      void fillBlockSizes();
    };

}
#endif
#define MUSIC_SUBCONNECTOR_HH
#endif
