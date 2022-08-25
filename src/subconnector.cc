/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2008, 2009, 2010, 2022 INCF
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

#include "music/subconnector.hh"

#include "music/debug.hh"

#if MUSIC_USE_MPI

#include "music/communication.hh"
#include "music/event_router.hh"

#include <cstring>
#include <algorithm>
#include <iostream>
#include <iterator>
#ifdef MUSIC_DEBUG
#include <cstdlib>
#endif

namespace MUSIC {

  Subconnector::Subconnector (MPI_Datatype type,
			      MPI_Comm intercomm_,
			      int remoteLeader,
			      int remoteRank,
			      int receiverRank,
			      int receiverPortCode)
    : type_ (type),
      intercomm (intercomm_),
      remoteRank_ (remoteRank),
      remoteWorldRank_ (remoteLeader + remoteRank),
      receiverRank_ (receiverRank),
      receiverPortCode_ (receiverPortCode),
      flushed(false)
  {
  }

  
  Subconnector::~Subconnector ()
  {
  }

  
  BufferingOutputSubconnector::BufferingOutputSubconnector (int elementSize)
    : buffer_ (elementSize)
  {
  }


  /********************************************************************
   *
   * Cont Subconnectors
   *
   ********************************************************************/

  ContOutputSubconnector::ContOutputSubconnector (//Synchronizer* synch_,
						  MPI_Comm intercomm_,
						  int remoteLeader,
						  int remoteRank,
						  int receiverPortCode_,
						  MPI_Datatype type)
    : Subconnector (type,
		    intercomm_,
		    remoteLeader,
		    remoteRank,
		    remoteRank,
		    receiverPortCode_),
      BufferingOutputSubconnector (0)
  {
  }

  
  void
  ContOutputSubconnector::initialCommunication (double param)
  {
    send ();
  }

  
  void
  ContOutputSubconnector::maybeCommunicate ()
  {
    if(!flushed)
      send ();
  }

  
  void
  ContOutputSubconnector::send ()
  {

    void* data;
    int size;
    buffer_.nextBlock (data, size);
    // NOTE: marshalling
    char* buffer = static_cast <char*> (data);

    while (size >= CONT_BUFFER_MAX)
      {

	MUSIC_LOGR ("Sending to rank " << remoteRank_);
	MPI_Ssend (buffer,
		   CONT_BUFFER_MAX / mpi_get_type_size (type_),
		   type_,
		   remoteRank_,
		   CONT_MSG,
		   intercomm);
	buffer += CONT_BUFFER_MAX;
	size -= CONT_BUFFER_MAX;
      }
    MUSIC_LOGR ("Last send to rank " << remoteRank_);
    MPI_Ssend (buffer,
	       size / mpi_get_type_size (type_),
	       type_,
	       remoteRank_,
	       CONT_MSG,
	       intercomm);
  }

  
  void
  ContOutputSubconnector::flush (bool& dataStillFlowing)
  {
    if (!flushed)
      {
	if (!buffer_.isEmpty ())
	  {
	    MUSIC_LOGR ("sending data remaining in buffers");
	    send ();
	    dataStillFlowing = true;
	  }
	else
	  {
	    char dummy;
	    MPI_Ssend (&dummy, 0, type_, remoteRank_, FLUSH_MSG, intercomm);
	    flushed = true;
	  }
      }
  }

  
  ContInputSubconnector::ContInputSubconnector (//Synchronizer* synch_,
						MPI_Comm intercomm,
						int remoteLeader,
						int remoteRank,
						int receiverRank,
						int receiverPortCode,
						MPI_Datatype type)
    : Subconnector (type,
		    intercomm,
		    remoteLeader,
		    remoteRank,
		    receiverRank,
		    receiverPortCode),
      InputSubconnector ()
  {
  }

  
  void
  ContInputSubconnector::initialCommunication (double initialBufferedTicks)
  {
    receive ();

    buffer_.fill (initialBufferedTicks);
  }


  void
  ContInputSubconnector::maybeCommunicate ()
  {
    if (!flushed)
      receive ();
  }

  
  void
  ContInputSubconnector::receive ()
  {
    char* data;
    MPI_Status status;
    int size, maxCount;
    maxCount = CONT_BUFFER_MAX / mpi_get_type_size (type_);
    do
      {
	data = static_cast<char*> (buffer_.insertBlock ());
	MUSIC_LOGR ("Receiving from rank " << remoteRank_);

	MPI_Recv (data,
		  maxCount,
		  type_,
		  remoteRank_,
		  MPI_ANY_TAG,
		  intercomm,
		  &status);
	if (status.MPI_TAG == FLUSH_MSG)
	  {
	    flushed = true;
	    MUSIC_LOGR ("received flush message");
	    return;
	  }
	MPI_Get_count (&status, type_, &size);
	buffer_.trimBlock (mpi_get_type_size (type_) * size);
      }
    while (size == maxCount);
  }

  
  void
  ContInputSubconnector::flush (bool& dataStillFlowing)
  {
    if (!flushed)
      {
	//MUSIC_LOGR ("receiving and throwing away data");
	receive ();
	if (!flushed)
	  dataStillFlowing = true;
      }
  }


  
  /********************************************************************
   *
   * Event Subconnectors
   *
   ********************************************************************/


  EventOutputSubconnector::EventOutputSubconnector (//Synchronizer* synch_,
						    MPI_Comm intercomm,
						    int remoteLeader,
						    int remoteRank,
						    int receiverPortCode)
    : Subconnector (MPI_BYTE,
		    intercomm,
		    remoteLeader,
		    remoteRank,
		    remoteRank,
		    receiverPortCode),
      BufferingOutputSubconnector (sizeof (Event))
  {
  }

  
  void
  EventOutputSubconnector::maybeCommunicate ()
  {
    send ();
  }

  
  void
  EventOutputSubconnector::maybeCommunicate (std::vector<MPI_Request> &requests)
  {
    send (requests);
  }

  
  void
  EventOutputSubconnector::send (std::vector<MPI_Request> &requests)
  {
    MUSIC_LOGRE ("ISend");
    void* data;
    int size;
    buffer_.nextBlock (data, size);
    char* buffer = static_cast <char*> (data);
    while (size >= SPIKE_BUFFER_MAX)
      {
	MPI_Request request;
	MPI_Isend (buffer,
		   SPIKE_BUFFER_MAX,
		   type_,
		   remoteRank_,
		   SPIKE_MSG,
		   intercomm,
		   &request);
	requests.push_back (request);
	buffer += SPIKE_BUFFER_MAX;
	size -= SPIKE_BUFFER_MAX;
      }
    MPI_Request request;
    MPI_Isend (buffer, size, type_, remoteRank_, SPIKE_MSG, intercomm, &request);
    requests.push_back (request);
  }

  
  void
  EventOutputSubconnector::send ()
  {
    MUSIC_LOGRE ("Ssend");
    void* data;
    int size;
    buffer_.nextBlock (data, size);
    char* buffer = static_cast <char*> (data);
    while (size >= SPIKE_BUFFER_MAX)
      {
	MPI_Ssend (buffer,
		   SPIKE_BUFFER_MAX,
		   type_,
		   remoteRank_,
		   SPIKE_MSG,
		   intercomm);
	buffer += SPIKE_BUFFER_MAX;
	size -= SPIKE_BUFFER_MAX;
      }
    MPI_Ssend (buffer, size, type_, remoteRank_, SPIKE_MSG, intercomm);
  }

  
  void
  EventOutputSubconnector::flush (bool& dataStillFlowing)
  {
    if (!flushed)
      {
	if (!buffer_.isEmpty ())
	  {
	    MUSIC_LOGR ("sending data remaining in buffers");
	    send ();
	    dataStillFlowing = true;
	  }
	/* remedius
	 * flushed flag was added since synchronous communication demands equal sends and receives
	 */
	else
	  {
	    Event* e = static_cast<Event*> (buffer_.insert ());
	    e->id = FLUSH_MARK;
	    send ();
	    flushed = true;
	  }
      }
  }


  EventInputSubconnector::EventInputSubconnector (//Synchronizer* synch_,
						  MPI_Comm intercomm,
						  int remoteLeader,
						  int remoteRank,
						  int receiverRank,
						  int receiverPortCode)
    : Subconnector (MPI_BYTE,
		    intercomm,
		    remoteLeader,
		    remoteRank,
		    receiverRank,
		    receiverPortCode),
      InputSubconnector ()
  {
  }


  EventInputSubconnectorGlobal::EventInputSubconnectorGlobal
  (//Synchronizer* synch_,
   MPI_Comm intercomm,
   int remoteLeader,
   int remoteRank,
   int receiverRank,
   int receiverPortCode,
   EventHandlerGlobalIndex* eh)
    : Subconnector (MPI_BYTE,
		    intercomm,
		    remoteLeader,
		    remoteRank,
		    receiverRank,
		    receiverPortCode),
      EventInputSubconnector (//synch_,
			      intercomm,
			      remoteLeader,
			      remoteRank,
			      receiverRank,
			      receiverPortCode),
      handleEvent (eh)
  {


  }


  //EventHandlerGlobalIndexDummy
  //EventInputSubconnectorGlobal::dummyHandler;


  EventInputSubconnectorLocal::EventInputSubconnectorLocal
  (//Synchronizer* synch_,
   MPI_Comm intercomm,
   int remoteLeader,
   int remoteRank,
   int receiverRank,
   int receiverPortCode,
   EventHandlerLocalIndex* eh)
    : Subconnector (MPI_BYTE,
		    intercomm,
		    remoteLeader,
		    remoteRank,
		    receiverRank,
		    receiverPortCode),
      EventInputSubconnector (
			      intercomm,
			      remoteLeader,
			      remoteRank,
			      receiverRank,
			      receiverPortCode),
      handleEvent (eh)
  {
  }


  //EventHandlerLocalIndexDummy
  //EventInputSubconnectorLocal::dummyHandler;


  void
  EventInputSubconnector::maybeCommunicate ()
  {
    if (!flushed )
      receive ();
  }

  bool
  EventInputSubconnectorGlobal::receive (char *data, int size)
  {
    Event* ev = (Event*) data;
    /* remedius
     * since the message can be of size 0 and contains garbage=FLUSH_MARK,
     * the check for the size of the message was added.
     */
    if (ev[0].id == FLUSH_MARK  && size > 0)
      {
	flushed = true;
	MUSIC_LOGR ("received flush message");
      }
    else
      {
	int nEvents = size / sizeof (Event);
	for (int i = 0; i < nEvents; ++i){
	  (*handleEvent) (ev[i].t, ev[i].id);
	}
      }
    return flushed;
  }

  
  // NOTE: isolate difference between global and local to avoid code repetition
  void
  EventInputSubconnectorGlobal::receive ()
  {
    char data[SPIKE_BUFFER_MAX];
    MPI_Status status;
    int size;
    //double starttime, endtime;
    //starttime = MPI_Wtime();

    do
      {
	MPI_Recv (data,
		  SPIKE_BUFFER_MAX,
		  type_,
		  remoteRank_,
		  SPIKE_MSG,
		  intercomm,
		  &status);

	MPI_Get_count (&status, type_, &size);
	Event* ev = (Event*) data;
	/* remedius
	 * since the message can be of size 0 and contains garbage=FLUSH_MARK,
	 * the check for the size of the message was added.
	 */
	if (ev[0].id == FLUSH_MARK  && size > 0)
	  {
	    flushed = true;
	    MUSIC_LOGR ("received flush message");
	    return;
	  }


	int nEvents = size / sizeof (Event);

	for (int i = 0; i < nEvents; ++i){
	  (*handleEvent) (ev[i].t, ev[i].id);
	}

      }
    while (size == SPIKE_BUFFER_MAX);
    //endtime = MPI_Wtime();
    //endtime = endtime-starttime;
    //if(tt < endtime)
    //tt = endtime;

  }


  void
  EventInputSubconnectorLocal::receive ()
  {
    MUSIC_LOGRE ("receive");
    char data[SPIKE_BUFFER_MAX];
    MPI_Status status;
    int size;
    do
      {
	MPI_Recv (data,
		  SPIKE_BUFFER_MAX,
		  type_,
		  remoteRank_,
		  SPIKE_MSG,
		  intercomm,
		  &status);
	MPI_Get_count (&status, type_, &size);
	Event* ev = (Event*) data;
	/* remedius
	 * since the message can be of size 0 and contains garbage=FLUSH_MARK,
	 * the check for the size of the message was added.
	 */
	if (ev[0].id == FLUSH_MARK && size > 0)
	  {
	    flushed = true;
	    return;
	  }
	int nEvents = size / sizeof (Event);
	for (int i = 0; i < nEvents; ++i)
	  (*handleEvent) (ev[i].t, ev[i].id);
      }
    while (size == SPIKE_BUFFER_MAX);
  }


  void
  EventInputSubconnector::flush (bool& dataStillFlowing)
  {
    if (!flushed)
      {
	//MUSIC_LOGRE ("receiving and throwing away data");
	receive ();
	if (!flushed)
	  dataStillFlowing = true;
      }
  }


  /*void
    EventInputSubconnectorGlobal::flush (bool& dataStillFlowing)
    {
    //handleEvent = &dummyHandler;
    EventInputSubconnector::flush (dataStillFlowing);
    }


    void
    EventInputSubconnectorLocal::flush (bool& dataStillFlowing)
    {
    //handleEvent = &dummyHandler;
    EventInputSubconnector::flush (dataStillFlowing);
    }*/



  /********************************************************************
   *
   * Message Subconnectors
   *
   ********************************************************************/

  MessageOutputSubconnector::MessageOutputSubconnector (//Synchronizer* synch_,
							MPI_Comm intercomm,
							int remoteLeader,
							int remoteRank,
							int receiverPortCode,
							FIBO* buffer)
    : Subconnector (MPI_BYTE,
		    intercomm,
		    remoteLeader,
		    remoteRank,
		    remoteRank,
		    receiverPortCode),
      buffer_ (buffer)
  {
  }


  void
  MessageOutputSubconnector::maybeCommunicate ()
  {
    if (!flushed)
      send ();
  }


  void
  MessageOutputSubconnector::send ()
  {
    void* data;
    int size;
    buffer_->nextBlock (data, size);
    // NOTE: marshalling
    char* buffer = static_cast <char*> (data);
    while (size >= MESSAGE_BUFFER_MAX)
      {
	MPI_Ssend (buffer,
		   MESSAGE_BUFFER_MAX,
		   type_,
		   remoteRank_,
		   MESSAGE_MSG,
		   intercomm);
	buffer += MESSAGE_BUFFER_MAX;
	size -= MESSAGE_BUFFER_MAX;
      }
    MPI_Ssend (buffer, size, type_, remoteRank_, MESSAGE_MSG, intercomm);
  }


  void
  MessageOutputSubconnector::flush (bool& dataStillFlowing)
  {
    if (!flushed)
      {
	if (!buffer_->isEmpty ())
	  {
	    MUSIC_LOGRE ("sending data remaining in buffers");
	    send ();
	    dataStillFlowing = true;
	  }
	else
	  {
	    char dummy;
	    MPI_Ssend (&dummy, 0, type_, remoteRank_, FLUSH_MSG, intercomm);
	  }
      }
  }


  MessageInputSubconnector::MessageInputSubconnector (//Synchronizer* synch_,
						      MPI_Comm intercomm,
						      int remoteLeader,
						      int remoteRank,
						      int receiverRank,
						      int receiverPortCode,
						      MessageHandler* mh)
    : Subconnector (MPI_BYTE,
		    intercomm,
		    remoteLeader,
		    remoteRank,
		    receiverRank,
		    receiverPortCode),
      handleMessage (mh)
  {
  }


  //MessageHandlerDummy
  //MessageInputSubconnector::dummyHandler;


  void
  MessageInputSubconnector::maybeCommunicate ()
  {
    if (!flushed)// && synch->communicate ())
      receive ();
  }


  void
  MessageInputSubconnector::receive ()
  {
    char data[MESSAGE_BUFFER_MAX];
    MPI_Status status;
    int size;
    do
      {
	MPI_Recv (data,
		  MESSAGE_BUFFER_MAX,
		  type_,
		  remoteRank_,
		  MPI_ANY_TAG,
		  intercomm,
		  &status);
	if (status.MPI_TAG == FLUSH_MSG)
	  {
	    flushed = true;
	    MUSIC_LOGRE ("received flush message");
	    return;
	  }
	MPI_Get_count (&status, type_, &size);
	int current = 0;
	while (current < size)
	  {
	    MessageHeader* header = static_cast<MessageHeader*>
	      (static_cast<void*> (&data[current]));
	    current += sizeof (MessageHeader);
	    (*handleMessage) (header->t (), &data[current], header->size ());
	    current += header->size ();
	  }
      }
    while (size == MESSAGE_BUFFER_MAX);
  }


  void
  MessageInputSubconnector::flush (bool& dataStillFlowing)
  {
    //handleMessage = &dummyHandler;
    if (!flushed)
      {
	//MUSIC_LOGRE ("receiving and throwing away data");
	receive ();
	if (!flushed)
	  dataStillFlowing = true;
      }
  }


  
  /********************************************************************
   *
   * Collective Subconnector
   *
   ********************************************************************/

  CollectiveSubconnector::CollectiveSubconnector (MPI_Comm intracomm)
    : intracomm_ (intracomm)
  {
  }


  CollectiveSubconnector::~CollectiveSubconnector ()
  {
  }

  
  void
  CollectiveSubconnector::allocAllgathervArrays ()
  {
    nProcesses = mpi_get_comm_size (intracomm_);
    ppBytes = new int[nProcesses];
    displ = new int[nProcesses];
  }

  
  void
  CollectiveSubconnector::freeAllgathervArrays ()
  {
    delete ppBytes;
    delete displ;  
  }

  
  void
  CollectiveSubconnector::maybeCommunicate ()
  {
    if (!flushed)
      communicate ();
  }

  
  int
  CollectiveSubconnector::calcCommDataSize (int local_data_size)
  {
    int dsize;
    //distributing the size of the buffer
    MPI_Allgather (&local_data_size, 1, MPI_INT, ppBytes, 1, MPI_INT, intracomm_);
    //could it be that dsize is more then unsigned int?
    dsize = 0;
    for(int i=0; i < nProcesses; ++i){
      displ[i] = dsize;
      dsize += ppBytes[i];
      //ppBytes[i] /= type_.Get_size();
      /*		  if(ppBytes[i] > max_size)
			  max_size = ppBytes[i];*/
    }
    return dsize;
  }

  
  /* remedius
   * current collective communication is realized through two times calls of
   * mpi allgather function: first time the size of the data is distributed,
   * the second time the data by itself is distributed.
   * Probably that could be not optimal for a huge # of ranks.
   */
  std::vector<char> CollectiveSubconnector::getCommData(char *cur_buff, int size)
  {
    unsigned int dsize;
    char *recv_buff;
    std::vector<char> commData;
    recv_buff = NULL;

    dsize = calcCommDataSize (size);

    if(dsize > 0){
      //distributing the data
      recv_buff = new char[dsize];
      MPI_Allgatherv (cur_buff, size, MPI_BYTE, recv_buff, ppBytes, displ, MPI_BYTE,
		      intracomm_);
      std::copy (recv_buff, recv_buff + dsize, std::back_inserter (commData));
      delete[] recv_buff;
    }
    return commData;
  }


  void
  EventOutputCollectiveSubconnector::setOutputBuffer (void* buffer,
						      unsigned int size)
  {
    router_->setOutputBuffer (buffer, size);
  }


  unsigned int
  EventOutputCollectiveSubconnector::dataSize ()
  {
    return router_->dataSize ();
  }


  void
  EventOutputCollectiveSubconnector::fillOutputBuffer ()
  {
    router_->fillOutputBuffer ();
  }


  void
  EventInputCollectiveSubconnector::processData (void* data, unsigned int size)
  {
    Event* e = static_cast<Event*> (data);
    while (size > 0)
      {
	router_->processEvent (e->t, e->id);
	++e;
	size -= sizeof (Event);
      }
  }


  void
  ContCollectiveSubconnector::maybeCommunicate()
  {
    CollectiveSubconnector::maybeCommunicate();
  }

  
  void
  ContCollectiveSubconnector::communicate()
  {
    int size, bSize, nBuffered;
    void *data;
    char *cur_buff, *dest_buff;
    std::vector<char> commData;
    if(flushed)
      return;
    ContOutputSubconnector::buffer_.nextBlock (data, size);
    commData = getCommData(static_cast <char*> (data), size);
    const int* displ = getDisplArr();
    //std::copy(commData.begin(),commData.end(),cur_buff);
    cur_buff = (char*)static_cast<void*> (&commData[0]);
    flushed = commData.size() == 0 ? true : false;
    nBuffered= commData.size() / width_; //the received data size is always multiple of port width
    for(int i =0; i < nBuffered ; ++i){
      bSize = 0;
      dest_buff = static_cast<char*> (ContInputSubconnector::buffer_.insertBlock ());
      for (std::multimap<int, Interval>::iterator it=
	     intervals_.begin() ; it != intervals_.end(); it++ ) //iterates all the intervals
	{
	  memcpy (dest_buff + bSize,
		  cur_buff
		  +displ[(*it).first]//+displacement of data for a current rank
		  +i*blockSizePerRank_[(*it).first]  //+displacement of the appropriate buffered chunk of data within current rank data block
		  +(*it).second.begin(), //+displacement of the requested data
		  (*it).second.end()); // length field is stored overlapping the end field
	  bSize += (*it).second.end();
	}
      ContInputSubconnector::buffer_.trimBlock (bSize);
    }
  }

  
  void
  ContCollectiveSubconnector::initialCommunication(double initialBufferedTicks)
  {
    communicate();
    fillBlockSizes();
    ContInputSubconnector::buffer_.fill (initialBufferedTicks);
  }

  
  void ContCollectiveSubconnector::fillBlockSizes()
  {
    std::map<int, Interval>::iterator it;
    const int* ppBytes = getSizeArr();
    for ( it=intervals_.begin() ; it != intervals_.end(); it++ ) //iterates all the intervals
      blockSizePerRank_[(*it).first]= ppBytes[(*it).first];
  }

  
  void
  ContCollectiveSubconnector::flush(bool& dataStillFlowing)
  {
    if (!flushed)
      {
	if (!ContOutputSubconnector::buffer_.isEmpty ())
	  {
	    MUSIC_LOGR ("sending data remaining in buffers");
	    maybeCommunicate ();
	    dataStillFlowing = true;
	  }
	else
	  {
	    //a sign for the end of communication will be an empty buffer
	    communicate ();
	    if(!flushed)
	      dataStillFlowing = true;
	  }
      }
  }
}
#endif
