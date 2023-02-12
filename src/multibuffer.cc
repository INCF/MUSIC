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

#include "music/multibuffer.hh"

#include "music/debug.hh"

#include <sstream>

#include <cstring>

extern "C" {
#include <assert.h>
}

#if MUSIC_USE_MPI

namespace MUSIC {

  /********************************************************************
   *
   * MultiBuffer
   *
   ********************************************************************/

  MultiBuffer::MultiBuffer (MPI_Comm comm,
			    int localLeader,
			    std::vector<Connector*>& connectors)
    : localLeader_ (localLeader)
  {
    //bool hang = true;
    //while (hang) ;
    MPI_Group worldGroup;
    MPI_Comm_group (MPI_COMM_WORLD, &worldGroup);
    MPI_Group localGroup;
    MPI_Comm_group (comm, &localGroup);
    int localSize = mpi_get_group_size (localGroup);

    // maps leaders to vectors mapping local ranks to COMM_WORLD ranks
    RankMap* rankMap = new RankMap ();
    setupRankMap (mpi_get_rank (comm), rankMap);
#if 0
    std::ostringstream ostr;
    ostr << "Rank " << mpi_get_rank (MPI_COMM_WORLD) << ": rankMap ";
    for (RankMap::iterator i = rankMap->begin ();
	 i != rankMap->end ();
	 ++i)
      {
	ostr << i->first << ": ";
	std::vector<int>& ranks = i->second;
	for (std::vector<int>::iterator j = ranks.begin ();
	     j != ranks.end ();
	     ++j)
	  ostr << *j << " ";
	ostr << " ";
      }
    std::cout << ostr.str () << std::endl;
#endif

    for (std::vector<Connector*>::iterator c = connectors.begin ();
	 c != connectors.end ();
	 ++c)
      {
	// supports multicommunication?
	if (!(*c)->idFlag ())
	  continue;
	
	CollectiveConnector* connector
	  = dynamic_cast<CollectiveConnector*> (*c);

	// We create InputSubconnectorInfo:s also for output connectors.
	// In this case s will be NULL.
	InputSubconnector* s
	  = dynamic_cast<InputSubconnector*> (connector->subconnector ());
	std::pair<InputConnectorMap::iterator, bool> res
	  = inputConnectorMap_.insert
	  (InputConnectorMap::value_type (connector,
					  InputSubconnectorInfo (s)));
	InputSubconnectorInfo& isi = res.first->second;

	// decide group leader and size
	int outputLeader;
	int outputSize;
	int inputLeader;
	int inputSize;
	if (connector->isInput ())
	  {
	    outputLeader = connector->remoteLeader ();
	    outputSize = connector->remoteNProcs ();
	    inputLeader = localLeader;
	    inputSize = localSize;
	  }
	else
	  {
	    outputLeader = localLeader;
	    outputSize = localSize;
	    inputLeader = connector->remoteLeader ();
	    inputSize = connector->remoteNProcs ();
	  }
	
	// setup BufferInfo array
	isi.setSize (outputSize);

	if (!connector->isInput ())
	  {
	    // create OutputSubconnectorInfo
	    OutputSubconnector* s
	      = dynamic_cast<OutputSubconnector*> (connector->subconnector ());
	    BufferInfo* bi = &*(isi.begin () + mpi_get_rank (comm));
	    outputConnectorMap_.insert
	      (OutputConnectorMap::value_type (connector,
					       OutputSubconnectorInfo (s, bi)));
	  }

	// register output group
	std::vector<int>& worldRanks = (*rankMap) [outputLeader];
	registerGroup (outputLeader, worldRanks);

	// setup Block array
	Blocks::iterator pos = getBlock (outputLeader);
	if (pos == block_.end () || pos->rank () != outputLeader)
	  {
	    // outputLeader not found in block_
	    // fill it in, creating one new Block for each rank
	    // with one BufferInfo per Block
	    int i = 0;
	    for (BufferInfos::iterator bi = isi.begin ();
		 bi != isi.end ();
		 ++bi, ++i)
	      {
		int worldRank = worldRanks[i];
		pos = getBlock (worldRank);
		int offset = pos - block_.begin ();
		block_.insert (pos, Block ());
		pos = block_.begin () + offset;
		pos->setRank (worldRank);
		pos->push_back (&*bi);
	      }
	  }
	else
	  {
	    // outputLeader's group of ranks already had Blocks in block_
	    // Insert one BufferInfo per Block
	    int i = 0;
	    for (BufferInfos::iterator bi = isi.begin ();
		 bi != isi.end ();
		 ++bi, ++i)
	      {
		getBlock (worldRanks[i])->push_back (&*bi);
	      }
	  }

	// register input group
	worldRanks = (*rankMap) [inputLeader];
	registerGroup (inputLeader, worldRanks);

	pos = getBlock (inputLeader);
	if (pos == block_.end () || pos->rank () != inputLeader)
	  {
	    // inputLeader's group of ranks were not represented in block_
	    // Create empty Block:s for them
	    for (int i = 0; i < inputSize; ++i)
	      {
		int worldRank = worldRanks[i];
		pos = getBlock (worldRank);
		int offset = pos - block_.begin ();
		block_.insert (pos, Block ());
		pos = block_.begin () + offset;
		pos->setRank (worldRank);
	      }
	  }

      }

#if 0
    {
      std::ostringstream ostr;
      ostr << "Rank " << mpi_get_rank (MPI_COMM_WORLD) << ": block_ ranks ";
      for (Blocks::iterator b = block_.begin (); b != block_.end (); ++b)
	ostr << b->rank () << ' ';
      std::cout << ostr.str () << std::endl;
    }
#endif

    delete rankMap;

    // setup Block and BufferInfo fields

    unsigned int start = 0;

    // reserve space for error block staging area
    for (Blocks::iterator b = block_.begin (); b != block_.end (); ++b)
      start = std::max (start, b->headerSize ());

    MPI_Allreduce (MPI_IN_PLACE, &start, 1, MPI_UNSIGNED, MPI_MAX, MPI_COMM_WORLD);

    errorBlockSize_ = start;

    for (Blocks::iterator b = block_.begin (); b != block_.end (); ++b)
      {
	b->setStart (start);
	start += sizeof (HeaderType); // error flag
	unsigned int size = b->headerSize ();
	if (size < errorBlockSize_)
	  {
	    start += errorBlockSize_ - size; // add some slack
	    size = errorBlockSize_;
	  }
	b->setSize (size);
	for (BufferInfoPtrs::iterator bi = b->begin (); bi != b->end (); ++bi)
	  {
	    start += sizeof (HeaderType); // data size field
	    (*bi)->setStart (start);
	    (*bi)->setSize (0);
	  }
      }

    // allocate buffer
    size_ = start;
    buffer_ = BufferType (malloc (size_));
    
    // clear flags in staging area
    clearFlags ();

    // write header fields into buffer
    for (Blocks::iterator b = block_.begin (); b != block_.end (); ++b)
      {
	b->clearBufferFlags (buffer_);
	for (BufferInfoPtrs::iterator bi = b->begin (); bi != b->end (); ++bi)
	  (*bi)->writeDataSize (buffer_, 0);
      }
  }


  void
  MultiBuffer::setupRankMap (int localRank, RankMap* rankMap)
  {
    int worldSize = mpi_get_comm_size (MPI_COMM_WORLD);
    int worldRank = mpi_get_rank (MPI_COMM_WORLD);
    std::vector<RankInfo> rankInfos (worldSize);
    rankInfos[worldRank] = RankInfo (localLeader_, localRank);
    MPI_Allgather (MPI_IN_PLACE, 0, MPI_DATATYPE_NULL,
		   &rankInfos.front (),
		   sizeof (RankInfo) / sizeof (int),
		   MPI_INT,
		   MPI_COMM_WORLD);
    for (int wr = 0; wr < worldSize; ++wr)
      {
	RankInfo& ri = rankInfos[wr];
	std::vector<int>& worldRanks = (*rankMap)[ri.leader];
	if (worldRanks.size () <= static_cast<unsigned int> (ri.localRank))
	  worldRanks.resize (ri.localRank + 1);
	worldRanks[ri.localRank] = wr;
      }
  }


  void
  MultiBuffer::registerGroup (unsigned int leader,
			      std::vector<int>& worldRanks)
  {
    if (groupMap_.find (leader) == groupMap_.end ())
      {
	Intervals& ivals = groupMap_[leader];
	assert (!worldRanks.empty ());
	std::vector<int>::iterator wr = worldRanks.begin ();
	int first = *wr;
	int last = first;
	for (++wr; wr != worldRanks.end (); ++wr)
	  if (*wr == last + 1)
	    last = *wr;
	  else
	    {
	      ivals.push_back (Interval (first, last));
	      first = last = *wr;
	    }
	ivals.push_back (Interval (first, last));
      }
  }


  unsigned int
  MultiBuffer::computeSize (bool twostage)
  {
    // compute required total size
    unsigned int summedSize = 0;
    unsigned int thisRankSize = 0;
    int thisRank = mpi_get_rank (MPI_COMM_WORLD);
    for (Blocks::iterator b = block_.begin (); b != block_.end (); ++b)
      {
	unsigned int size;
	if (!twostage && !b->errorFlag (buffer_))
	  size = b->size ();
	else
	  {
	    // this block requires more space
	    unsigned int blockSize = sizeof (HeaderType); // error flag
	    int i = 0;
	    for (BufferInfoPtrs::iterator bi = b->begin ();
		 bi != b->end ();
		 ++bi, ++i)
	      {
		blockSize += sizeof (HeaderType); // size field
		unsigned int requested = b->requestedDataSize (buffer_, i);
		if (requested > (*bi)->size ())
		  {
		    blockSize += requested;
		  }
		else
		  blockSize += (*bi)->size ();
	      }
	    size = std::max (errorBlockSize_, blockSize);
	  }
	summedSize += size;
	if (b->rank () == thisRank)
	  thisRankSize = size;
      }
    return thisRankSize + summedSize;
  }


  void
  MultiBuffer::restructure (bool twostage)
  {
    unsigned int size = computeSize (twostage);
    // resize multi-buffer
    if (size > size_)
      {
	buffer_ = BufferType (realloc (buffer_, size));
	size_ = size;
      }

    // relocate buffers
    unsigned int newStart = size;
    for (Blocks::reverse_iterator b = block_.rbegin ();
	 b != block_.rend ();
	 ++b)
      {
	if (!twostage && !b->errorFlag (buffer_))
	  {
	    // move entire block with one memmove
	    newStart -= b->size ();
	    unsigned int oldStart = b->start ();
	    unsigned int offset = newStart - oldStart;
	    if (offset == 0)
	      break; // we know that there are no further error flags set
	    memmove (buffer_ + newStart, buffer_ + oldStart, b->size ());
	    b->setStart (newStart);
	    for (BufferInfoPtrs::iterator bi = b->begin ();
		 bi != b->end ();
		 ++bi)
	      (*bi)->setStart ((*bi)->start () + offset);
	  }
	else
	  {
	    unsigned int lastStart = newStart;
	    // at least one of the buffers requires more storage
	    int i = b->nBuffers () - 1;
	    for (BufferInfoPtrs::reverse_iterator bi
		   = b->rbegin ();
		 bi != b->rend ();
		 ++bi, --i)
	      {
		unsigned int oldPos = (*bi)->start () - sizeof (HeaderType);
		unsigned int oldSize = (*bi)->size ();
		unsigned int requested = b->requestedDataSize (buffer_, i);
		if (requested > oldSize)
		  (*bi)->setSize (requested);
		newStart -= (*bi)->size ();
		(*bi)->setStart (newStart);
		newStart -= sizeof (HeaderType);
		// only touch buffer contents if the block belongs to
		// current rank => error staging area is used for
		// requested buffer sizes and buffer contains output
		// data
		if (b->rank () == mpi_get_rank (MPI_COMM_WORLD)
		    && newStart > oldPos)
		  memmove (buffer_ + newStart,
			   buffer_ + oldPos,
			   oldSize + sizeof (HeaderType));
	      }
	    newStart -= sizeof (HeaderType); // error flag
	    *headerPtr (buffer_ + newStart)
	      = *headerPtr (buffer_ + b->start ());
	    unsigned int blockSize = lastStart - newStart;
	    if (blockSize < errorBlockSize_)
	      {
		newStart -= errorBlockSize_ - blockSize;
		blockSize = errorBlockSize_;
	      }
	    b->setStart (newStart);
	    b->setSize (blockSize);
	    b->clearBufferErrorFlag (buffer_);
	  }
      }
    clearErrorFlag ();

    // update all existing multiConnectors
    for (std::vector<Updateable*>::iterator c = multiConnectors.begin ();
	 c != multiConnectors.end ();
	 ++c)
      (*c)->update ();
  }

  
  MultiBuffer::Blocks::iterator
  MultiBuffer::getBlock (int rank)
  {
    // binary search (modified from Wikipedia)
    int imin = 0;
    int imax = block_.size () - 1;
    while (imax >= imin)
      {
	/* calculate the midpoint for roughly equal partition */
	int imid = (imin + imax) / 2;
 
	// determine which subarray to search
	if (block_[imid].rank () < rank)
	  // change min index to search upper subarray
	  imin = imid + 1;
	else if (block_[imid].rank () > rank)
	  // change max index to search lower subarray
	  imax = imid - 1;
	else
	  // block found at index imid
	  return block_.begin () + imid;
      }
    // rank not found---return position where rank should be inserted
    return block_.begin () + imin;
  }

  MultiBuffer::InputSubconnectorInfo*
  MultiBuffer::getInputSubconnectorInfo (Connector* connector)
  {
    InputConnectorMap::iterator pos = inputConnectorMap_.find (connector);
    assert (pos != inputConnectorMap_.end ());
    return &pos->second;
  }


  MultiBuffer::OutputSubconnectorInfo*
  MultiBuffer::getOutputSubconnectorInfo (Connector* connector)
  {
    OutputConnectorMap::iterator pos = outputConnectorMap_.find (connector);
    assert (pos != outputConnectorMap_.end ());
    return &pos->second;
  }


  void
  MultiBuffer::dumpBlocks ()
  {
    for (Blocks::iterator b = block_.begin (); b != block_.end (); ++b)
      std::cout << b->rank () << std::endl;
  }


  /********************************************************************
   *
   * MultiConnector
   *
   ********************************************************************/

  MultiConnector::MultiConnector (MultiBuffer* multiBuffer)
    : multiBuffer_ (multiBuffer),
      recvcountInvalid_ (false)
  {
    connectorCode_ = ConnectorInfo::allocPortCode ();
    buffer_ = multiBuffer_->buffer ();
    groupMap_ = new GroupMap;
    multiBuffer_->addMultiConnector (this);
  }

  //*fixme* code repetition
  MultiConnector::MultiConnector (MultiBuffer* multiBuffer,
				  std::vector<Connector*>& connectors)
    : multiBuffer_ (multiBuffer),
      recvcountInvalid_ (false)
  {
    buffer_ = multiBuffer_->buffer ();
    groupMap_ = new GroupMap;
    multiBuffer_->addMultiConnector (this);
    for (std::vector<Connector*>::iterator c = connectors.begin ();
	 c != connectors.end ();
	 ++c)
      add (*c);
    initialize ();
  }

  void
  MultiConnector::mergeGroup (int leader, bool isInput)
  {
    if (groupMap_->find (leader) == groupMap_->end ())
      {
	MCGroupInfo& g = (*groupMap_)[leader];
	g.worldRankIntervals = &multiBuffer_->getWorldRankIntervals (leader);
	g.blank = isInput;
      }
    else
      // Only groups which provide output communicate.  Groups which
      // only have inputs are blanked.  "Blanked" means that the
      // corresponding recvcount in Allgather is 0.
      (*groupMap_)[leader].blank &= isInput;
  }

  void
  MultiConnector::add (Connector* connector)
  {
    if (connector->isInput ())
      {
	// The following has to be done here so that multiConnector
	// groups are created the same way in all applications.
	//
	// BUT: Do we need to sort the connectors? We do if the
	// scheduler can deliver connectors in different order in
	// different applications.
	connectorIds_.push_back (std::make_pair (connector->remoteLeader (),
						 multiBuffer_->localLeader ()));
	mergeGroup (multiBuffer_->localLeader (), true);
	mergeGroup ((connector)->remoteLeader (), false);
	inputSubconnectorInfo_.push_back (multiBuffer_
					  ->getInputSubconnectorInfo (connector));
      }
    else
      {
	connectorIds_.push_back (std::make_pair (multiBuffer_->localLeader (),
						 connector->remoteLeader ()));
	mergeGroup ((connector)->remoteLeader (), true);
	mergeGroup (multiBuffer_->localLeader (), false);
	outputSubconnectorInfo_.push_back (multiBuffer_
					   ->getOutputSubconnectorInfo (connector));
      }
  }

  void
  MultiConnector::initialize ()
  {
    bool isContiguous = true;
    {
#ifdef MUSIC_DEBUG
      std::ostringstream ostr;
      ostr << "Rank " << mpi_get_rank (MPI_COMM_WORLD) << ": Create ";
#endif
      int nRanges = 0;
      for (GroupMap::iterator g = groupMap_->begin ();
	   g != groupMap_->end ();
	   ++g)
	nRanges += g->second.worldRankIntervals->size ();

      int (*range)[3] = new int[nRanges][3];
      int i = 0;
      for (GroupMap::iterator g = groupMap_->begin ();
	   g != groupMap_->end ();
	   ++g)
	{
	  int leader = g->first;
	  Intervals& ivals = *g->second.worldRankIntervals;
#ifdef MUSIC_DEBUG
	  ostr << leader << ", ";
#endif
	  int size = 0;
	  int next = ivals[0].first;
	  for (Intervals::iterator ival = ivals.begin ();
	       ival != ivals.end ();
	       ++ival)
	    {
	      range[i][0] = ival->first;
	      range[i][1] = ival->last;
	      range[i][2] = 1;
	      ++i;
	      if (ival->first != next)
		isContiguous = false;
	      next = ival->last + 1;
	      size += ival->last - ival->first + 1;
	    }
	  g->second.size = size;
	}
#ifdef MUSIC_DEBUG
      ostr << std::endl;
      std::cout << ostr.str () << std::flush;
#endif
      MPI_Group worldGroup;
      MPI_Comm_group (MPI_COMM_WORLD, &worldGroup);
      MPI_Group_range_incl (worldGroup, nRanges, range, &group_);
      delete[] range;
    }

    sort (connectorIds_.begin (), connectorIds_.end ());
    std::ostringstream idstr_;
    std::vector<std::pair<int, int> >::iterator cid = connectorIds_.begin ();
    idstr_ << cid->first << cid->second;
    ++cid;
    for (; cid != connectorIds_.end (); ++cid)
      idstr_ << ':' << cid->first << cid->second;
    id_ = "mc" + idstr_.str ();

    if (!isContiguous
	|| mpi_get_group_size (group_) < mpi_get_comm_size (MPI_COMM_WORLD))
      MPI_Comm_create (MPI_COMM_WORLD, group_, &comm_);
    else
      comm_ = MPI_COMM_WORLD;
    MPI_Barrier (MPI_COMM_WORLD);

    std::vector<int> ranks (size ());
    std::vector<int> indices (size ());
    for (int rank = 0; rank < size (); ++rank)
      ranks[rank] = rank;
    MPI_Group_translate_ranks (group_, size (), &ranks[0],
			       mpi_get_group (MPI_COMM_WORLD), &indices[0]);
#ifdef MUSIC_TWOSTAGE_ALLGATHER
    twostage_ = true;
#endif
    for (int rank = 0; rank < size (); ++rank)
      {
	Blocks::iterator b = multiBuffer_->getBlock (indices[rank]);
#ifdef MUSIC_DEBUG
	if (b == multiBuffer_->blockEnd () && mpi_get_rank (MPI_COMM_WORLD) == 0)
	  {
	    std::cout << "asked for rank " << indices[rank] << " among:" << std::endl;
	    multiBuffer_->dumpBlocks ();
	  }
#endif
	assert (b != multiBuffer_->blockEnd ());
	block_.push_back (&*b);
	
#ifdef MUSIC_TWOSTAGE_ALLGATHER
	// *fixme* need better criterion in the case of MC:s with
	// proxy connectors
	if (b->nBuffers () > 1)
	  twostage_ = false;
	else if (twostage_ && b->nBuffers () > 0)
	  {
	    BufferInfo* bi = *b->begin ();
	    // Should store this information in separate data structure
	    bi->setRank (rank);
	  }
#endif
      }

    recvcounts_ = new int[size ()];
    displs_ = new int[size ()];

    blank_ = new bool[size ()];
    int i = 0;
    for (GroupMap::iterator g = groupMap_->begin ();
	 g != groupMap_->end ();
	 ++g)
      {
	int leader = g->first;
	int size = (*groupMap_)[leader].size;
	int limit = i + size;
	for (; i < limit; ++i)
	  blank_[i] = g->second.blank;
      }
    assert (i == size ());
    delete groupMap_;

    restructuring_ = true;
    update ();
    restructuring_ = false;
  }


  void
  MultiConnector::update ()
  {
    buffer_ = multiBuffer_->buffer ();
    for (OutputSubconnectorInfos::iterator osi
	   = outputSubconnectorInfo_.begin ();
	 osi != outputSubconnectorInfo_.end ();
	 ++osi)
      {
	unsigned int dataSize = (*osi)->subconnector ()->dataSize ();
	BufferInfo* bi = (*osi)->bufferInfo ();
	bi->writeDataSize (buffer_, dataSize);
	(*osi)->subconnector ()->setOutputBuffer (buffer_
						  + bi->start (),
						  bi->size ());
      }
    for (int r = 0; r < size (); ++r)
      {
	Block* block = block_[r];
#ifdef MUSIC_TWOSTAGE_ALLGATHER
	if (twostage_)
	  {
	    if (block->begin () != block->end ())
	      displs_[r] = (*block->begin ())->start ();
	    else
	      displs_[r] = 0;
	  }
	else
#endif
	  {
	    int newRecvcount = blank_[r] ? 0 : block->size ();
	    if (!restructuring_ && r == rank () && newRecvcount > recvcounts_[r])
	      // Another MultiConnector has caused MultiBuffer
	      // restructuring.  We need to postpone modifying recvcount
	      // until our partner knows the new size.
	      recvcountInvalid_ = true;
	    else
	      recvcounts_[r] = newRecvcount;
	    displs_[r] = block->start ();
	  }
      }
  }


  bool
  MultiConnector::writeSizes ()
  {
    bool dataFits = true;
    for (OutputSubconnectorInfos::iterator osi
	   = outputSubconnectorInfo_.begin ();
	 osi != outputSubconnectorInfo_.end ();
	 ++osi)
      {
	(*osi)->subconnector ()->nextBlock ();
	unsigned int dataSize = (*osi)->subconnector ()->dataSize ();
	if (!recvcountInvalid_)
	  (*osi)->bufferInfo ()->writeDataSize (buffer_, dataSize);
	if (dataSize > (*osi)->bufferInfo ()->size ())
	    dataFits = false;
      }
    if (recvcountInvalid_ // another MultiConnector caused restructuring
	|| !dataFits)
      {
	displs_[rank ()] = 0; // error block staging area
	multiBuffer_->setErrorFlag ();
	int i = 0;
	for (OutputSubconnectorInfos::iterator osi
	       = outputSubconnectorInfo_.begin ();
	     osi != outputSubconnectorInfo_.end ();
	     ++osi, ++i)
	  {
	    unsigned int size = (*osi)->bufferInfo ()->size ();
	    unsigned int dataSize = (*osi)->subconnector ()->dataSize ();
	    multiBuffer_->writeRequestedDataSize (i, std::max (size, dataSize));
	  }
	return false;
      }
    return true;
  }


  void
  MultiConnector::fillBuffers ()
  {
    for (OutputSubconnectorInfos::iterator osi
	   = outputSubconnectorInfo_.begin ();
	 osi != outputSubconnectorInfo_.end ();
	 ++osi)
      (*osi)->subconnector ()->fillOutputBuffer ();
  }


  void
  MultiConnector::processInput ()
  {
    for (InputSubconnectorInfoPtrs::iterator isi
	   = inputSubconnectorInfo_.begin ();
	 isi != inputSubconnectorInfo_.end ();
	 ++isi)
      {
	InputSubconnector* subconnector = (*isi)->subconnector ();
	for (BufferInfos::iterator bi = (*isi)->begin ();
	     bi != (*isi)->end ();
	     ++bi)
	  subconnector->processData (buffer_ + bi->start (),
				     bi->readDataSize (buffer_));
      }
  }


#ifdef MUSIC_TWOSTAGE_ALLGATHER
  void
  MultiConnector::processReceived ()
  {
    for (InputSubconnectorInfoPtrs::iterator isi
	   = inputSubconnectorInfo_.begin ();
	 isi != inputSubconnectorInfo_.end ();
	 ++isi)
      {
	InputSubconnector* subconnector = (*isi)->subconnector ();
	for (BufferInfos::iterator bi = (*isi)->begin ();
	     bi != (*isi)->end ();
	     ++bi)
	  subconnector->processData (buffer_ + bi->start (),
				     recvcounts_[bi->rank ()]);
      }
  }


  void
  MultiConnector::checkRestructure ()
  {
    bool dataFits = true;
    doAllgather_ = false;
    int r = 0;
    for (BlockPtrs::iterator b = block_.begin ();
	 b != block_.end ();
	 ++r, ++b)
      {
	int size = recvcounts_[r];
	if (size)
	  doAllgather_ = true;
	if (size & TWOSTAGE_FINALIZE_FLAG)
	  {
	    block_[r]->setFinalizeFlag (buffer_);
	    recvcounts_[r] &= ~TWOSTAGE_FINALIZE_FLAG;
	  }
	BufferInfoPtrs::iterator bipi = (*b)->begin ();
	if (bipi != (*b)->end ())
	  {
	    BufferInfo* bi = *bipi;
	    if (size > bi->size ())
	      dataFits = false;
	  }
      }
    if (!dataFits)
      {
	int r = 0;
	for (BlockPtrs::iterator b = block_.begin ();
	     b != block_.end ();
	     ++r, ++b)
	  {
	    BufferInfoPtrs::iterator bipi = (*b)->begin ();
	    if (bipi != (*b)->end ())
	      {
		int size = recvcounts_[r];
		BufferInfo* bi = *bipi;
		if (size < bi->size ())
		  size = bi->size ();
		bi->writeDataSize (buffer_, size);
	      }
	  }
	BufferInfoPtrs::iterator bipi = block_[rank()]->begin ();
	int bsize = 0;
	if (bipi != block_[rank ()]->end ())
	  bsize = (*bipi)->size ();
	multiBuffer_->writeRequestedDataSize (0,
					      std::max (recvcounts_[rank ()],
							bsize));
	multiBuffer_->restructure (true);
      }
  }  
#endif


  static void
  dumprecvc (std::string id, int* recvc, int* displs, int n)
  {
    std::ostringstream ostr;
#if 1
    ostr << "Rank " << mpi_get_rank (MPI_COMM_WORLD) << ": "
	 << id << ": Allgather " << *recvc;
    for (int i = 1; i < n; ++i)
      ostr << ", " << recvc[i];
#else
    ostr << "Rank " << mpi_get_rank (MPI_COMM_WORLD) << ": "
	 << id << ": Allgather "
	 << *displs << ':' << *recvc;
    for (int i = 1; i < n; ++i)
      ostr << ", " << displs[i] << ':' << recvc[i];
#endif
    ostr << std::endl;
    std::cout << ostr.str () << std::flush;
  }


  void
  MultiConnector::tick ()
  {
#ifdef MUSIC_TWOSTAGE_ALLGATHER
    if (twostage_)
      {
	// *fixme* nextBlock ()
	OutputSubconnectorInfos::iterator osi = outputSubconnectorInfo_.begin ();
	int recvc;
	if (osi != outputSubconnectorInfo_.end ())
	  recvc = (*osi)->subconnector ()->dataSize ();
	else
	  recvc = 0;
	if (block_[rank ()]->finalizeFlag (buffer_))
	  recvc |= TWOSTAGE_FINALIZE_FLAG;
	recvcounts_[rank ()] = recvc;
	MPI_Allgather (MPI_IN_PLACE, 0, MPI_DATATYPE_NULL,
		       recvcounts_, 1, MPI_INT,
		       comm_);
	checkRestructure (); // sets doAllgather_
	if (doAllgather_)
	  {
	    fillBuffers ();
#ifdef MUSIC_DEBUG
	    dumprecvc (id_, recvcounts_, displs_, comm_.Get_size ());
#endif
	    MPI_Allgatherv (MPI_IN_PLACE, 0, MPI_DATATYPE_NULL,
			    buffer_, recvcounts_, displs_, MPI_BYTE,
			    comm_);
	    processReceived ();
	  }
      }
    else
#endif
      {
	if (writeSizes ())
	  // Data will fit
	  fillBuffers ();
#ifdef MUSIC_DEBUG
	dumprecvc (id_, recvcounts_, displs_, comm_.Get_size ());
#endif
	MPI_Allgatherv (MPI_IN_PLACE, 0, MPI_DATATYPE_NULL,
			buffer_, recvcounts_, displs_, MPI_BYTE,
			comm_);
	for (BlockPtrs::iterator b = block_.begin ();
	     b != block_.end ();
	     ++b)
	  if ((*b)->errorFlag (buffer_))
	    {
	      restructuring_ = true;
	      multiBuffer_->restructure (false);
	      restructuring_ = false;
	      recvcountInvalid_ = false;
	      fillBuffers ();
#ifdef MUSIC_DEBUG
	      dumprecvc (id_, recvcounts_, displs_, comm_.Get_size ());
#endif
	      MPI_Allgatherv (MPI_IN_PLACE, 0, MPI_DATATYPE_NULL,
			      buffer_, recvcounts_, displs_, MPI_BYTE,
			      comm_);
	      break;
	    }
	processInput ();
      }
  }


  bool
  MultiConnector::isFinalized ()
  {
    bool finalized = true;
    for (int i = 0; i < size (); ++i)
      if (!blank_[i] && !block_[i]->finalizeFlag (buffer_))
	finalized = false;
    return finalized;
  }


  void
  MultiConnector::finalize ()
  {
    if (!blank_[rank ()])
      block_[rank ()]->setFinalizeFlag (buffer_);
  }

}

#endif // MUSIC_USE_MPI
