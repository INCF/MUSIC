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

#include "music/spatial.hh" // Must be included first on BG/Ls

#include "music/debug.hh"

#if MUSIC_USE_MPI

#include <sstream>

#include "music/error.hh"
#include "music/communication.hh"
#include "music/connector.hh" // used only for debugging

namespace MUSIC {

  NegotiationIterator::BufferTraversal::BufferTraversal
  (std::vector<NegotiationIntervals>& buffers_)
    : buffers (buffers_), buffer (0), interval (0)
  {
    findInterval ();
  }


  void
  NegotiationIterator::BufferTraversal::findInterval ()
  {
    while (buffer < buffers.size () && interval == buffers[buffer].size ())
      {
	// Check next buffer
	++buffer;
	interval = 0;
      }
  }
  

  bool
  NegotiationIterator::BufferTraversal::end ()
  {
    return buffer == buffers.size ();
  }


  void
  NegotiationIterator::BufferTraversal::operator++ ()
  {
    ++interval;
    findInterval ();
  }


  SpatialNegotiationData*
  NegotiationIterator::BufferTraversal::dereference ()
  {
    return &buffers[buffer][interval];
  }


  void
  NegotiationIterator::init (Implementation* impl)
  {
    implementation_ = impl;
    end_ = false;
    ++*this;
  }


  NegotiationIterator::~NegotiationIterator ()
  {
    delete implementation_;
  }

  
  NegotiationIterator::NegotiationIterator (const NegotiationIterator& i)
    : implementation_ (i.implementation_->copy ()),
      current_ (i.current_),
      end_ (i.end_)
  {
  }

  
  const NegotiationIterator&
  NegotiationIterator::operator= (const NegotiationIterator& i)
  {
    delete implementation_;
    implementation_ = i.implementation_->copy ();
    current_ = i.current_;
    end_ = i.end_;
    return *this;
  }

  
  NegotiationIterator&
  NegotiationIterator::operator++ ()
  {
    if (implementation_->end ())
      end_ = true;
    else
      {
	current_ = *implementation_->dereference ();
	++*implementation_;
	while (!implementation_->end ()
	       && implementation_->dereference ()->begin () == current_.end ()
	       && implementation_->dereference ()->local () == current_.local ()
	       && implementation_->dereference ()->rank () == current_.rank ())
	  {
	    // join intervals
	    current_.setEnd (implementation_->dereference ()->end ());
	    ++*implementation_;
	  }
      }
    return *this;
  }

  
  SpatialNegotiationData*
  NegotiationIterator::operator-> ()
  {
    return &current_;
  }


  SpatialNegotiator::SpatialNegotiator (IndexMap* ind, Index::Type type_, MPI_Comm c)
  : comm(c), indices (ind), type (type_)
  {

	  nProcesses = mpi_get_comm_size (comm);
	  localRank = mpi_get_rank (comm);
	  negotiateWidth ();

  }



  void
  SpatialNegotiator::negotiateWidth ()
  {

    // First determine local least upper bound and width
    int u = 0;
    int w = 0;
    for (IndexMap::iterator i = indices->begin ();
	 i != indices->end ();
	 ++i)
      {
	if (i->end () > u)
	  u = i->end ();
	w += i->end () - i->begin ();

      }

    // Now take maximum over all processes
    std::vector<int> m (nProcesses);
    MPI_Allgather (&u, 1, MPI_INT, &m[0], 1, MPI_INT, comm);
    for (unsigned int i = 0; i < nProcesses; ++i)
      if (m[i] > u)
	u = m[i];
    width = u;
    MPI_Allgather (&w, 1, MPI_INT, &m[0], 1, MPI_INT, comm);
    for (unsigned int i = 0; i < nProcesses; ++i)
      if (m[i] > w)
	w = m[i];

    maxLocalWidth_ = w;


  }


  void
  SpatialOutputNegotiator::negotiateWidth ()
  {
    if (localRank == 0)
      {
	// Receiver might need to know sender width
	MPI_Send (&width, 1, MPI_INT, 0, WIDTH_MSG, intercomm);
	int remoteWidth;
	MPI_Recv (&remoteWidth, 1, MPI_INT, 0, WIDTH_MSG, intercomm, MPI_STATUS_IGNORE);
	if (remoteWidth != width)
	  {
	    std::ostringstream msg;
	    msg << "sender and receiver width mismatch ("
		<< width << " != " << remoteWidth << ")";
	    error (msg.str ());
	  }
      }
  }


  void
  SpatialInputNegotiator::negotiateWidth ()
  {
    if (localRank == 0)
      {
	int remoteWidth;
	MPI_Recv (&remoteWidth, 1, MPI_INT, 0, WIDTH_MSG, intercomm, MPI_STATUS_IGNORE);
	// NOTE: For now, the handling of Index::WILDCARD_MAX is a bit
	// incomplete since, if there is any index interval on the
	// receiver side with index larger than the sender side width,
	// we will still choose sender side width as receiver side
	// width.
	/* remedius
	 * temporal fix is width = remoteWidth; shouldn't be like this!!!
	 */
	if (width == Index::WILDCARD_MAX || width < remoteWidth )
	  width = remoteWidth;
	MPI_Send (&width, 1, MPI_INT, 0, WIDTH_MSG, intercomm);
      }
    // Broadcast result (if we used a wildcard)
    MPI_Bcast (&width, 1, MPI_INT, 0, comm);
    if (maxLocalWidth_ == Index::WILDCARD_MAX)
      maxLocalWidth_ = width;
  }

  
  NegotiationIterator
  SpatialNegotiator::canonicalDistribution (int width, int nProcesses)
  {
    class Wrapper : public NegotiationIterator::Implementation {
      int i;
      int r;
      int w;
      int nPerProcess;
      SpatialNegotiationData data;
    public:
      Wrapper (int width, int nProcesses)
	: i (0), r (0), w (width)
      {
	nPerProcess = w / nProcesses;
	if (w % nProcesses > 0)
	  ++nPerProcess;
      }
      bool end () { return i >= w; }
      void operator++ () { ++r; i += nPerProcess; }
      SpatialNegotiationData* dereference ()
      {
	int high = std::min (i + nPerProcess, w);
	data = SpatialNegotiationData (IndexInterval (i, high, 0), r, 0);
	return &data;
      }
      Implementation* copy ()
      {
	return new Wrapper (*this);
      }
    };

    return NegotiationIterator (new Wrapper (width, nProcesses));
  }

  
  NegotiationIterator
  SpatialNegotiator::wrapIntervals (IndexMap::iterator beg,
				    IndexMap::iterator end,
				    Index::Type type,
				    int rank)
  {
    class Wrapper : public NegotiationIterator::Implementation {
    protected:
      SpatialNegotiationData data;
      IndexMap::iterator i;
    private:
      IndexMap::iterator end_;
    protected:
      int rank_;
      int cur_displ_;
    public:
      Wrapper (IndexMap::iterator beg,
	       IndexMap::iterator end,
	       int rank)
	: i (beg), end_ (end), rank_ (rank)
      {
    	  cur_displ_ = 0;
      }
      bool end () { return i == end_; }
      void operator++ () {
    	  cur_displ_ +=(i->end() - i->begin());
      ++i;
      if(end())
    	  cur_displ_ = 0;}
    };

    class GlobalWrapper : public Wrapper {
    public:
      GlobalWrapper (IndexMap::iterator beg,
		     IndexMap::iterator end,
		     int rank)
	: Wrapper (beg, end, rank)
      {
      }
      SpatialNegotiationData* dereference ()
      {
	data = SpatialNegotiationData (*i, rank_, cur_displ_);
	data.setLocal (0);
	return &data;
      }
      Implementation* copy ()
      {
	return new GlobalWrapper (*this);
      }
    };
  
    class LocalWrapper : public Wrapper {
    public:
      LocalWrapper (IndexMap::iterator beg,
		    IndexMap::iterator end,
		    int rank)
	: Wrapper (beg, end, rank)
      {
      }
      SpatialNegotiationData* dereference ()
      {
	data = SpatialNegotiationData (*i, rank_, cur_displ_);
	return &data;
      }
      Implementation* copy ()
      {
	return new LocalWrapper (*this);
      }
    };
  
    Wrapper* w;
    if (type == Index::GLOBAL)
      w = new GlobalWrapper (beg, end, rank);
    else
      w = new LocalWrapper (beg, end, rank);
    return NegotiationIterator (w);
  }

  
  // Compute intersection intervals between source and dest.  Store
  // the resulting intervals with rank from source in buffer
  // belonging to rank in dest.
  void
  SpatialNegotiator::intersectToBuffers
  (std::vector<NegotiationIntervals>& source,
   std::vector<NegotiationIntervals>& dest,
   std::vector<NegotiationIntervals>& buffers)
  {
    // Cleanup old buffer content
    for (std::vector<NegotiationIntervals>::iterator i = buffers.begin ();
	 i != buffers.end ();
	 ++i)
      i->clear ();
    for (std::vector<NegotiationIntervals>::iterator d = dest.begin ();
	 d != dest.end ();
	 ++d)
      for (std::vector<NegotiationIntervals>::iterator s = source.begin ();
	   s != source.end ();
	   ++s)
	intersectToBuffers2 (*s, *d, buffers);
  }

  
  void
  SpatialNegotiator::intersectToBuffers
  (NegotiationIterator source,
   NegotiationIterator dest,
   std::vector<NegotiationIntervals>& buffers)
  {
    // Cleanup old buffer content
    for (std::vector<NegotiationIntervals>::iterator i = buffers.begin ();
	 i != buffers.end ();
	 ++i)
      i->clear ();
    intersectToBuffers2 (source, dest, buffers);
  }
    
    
  void
  SpatialNegotiator::intersectToBuffers2
  (NegotiationIterator source,
   NegotiationIterator dest,
   std::vector<NegotiationIntervals>& buffers)
  {
    while (!source.end () && !dest.end ())
      {
	MUSIC_LOGX("(" << source->begin () << ", "
		    << source->end () << ", "
		    << source->local () << ", "
		    << source->rank () << ") and ("
		    << dest->begin () << ", "
		    << dest->end () << ", "
		    << dest->local () << ", "
		    << dest->rank () << ")");
	if (source->begin () < dest->begin ())
	  if (dest->begin () < source->end ())
	    if (dest->end () < source->end ())
	      {
		// NOTE: put into helper function to get overview
		SpatialNegotiationData d (dest->begin (),
					  dest->end (),
					  dest->local () - source->local (),
					  source->rank (),
					  source->displ()+(dest->begin()-source->begin())
					  );
		buffers[dest->rank ()].push_back (d);
		++dest;
	      }
	    else
	      {
		SpatialNegotiationData d (dest->begin (),
					  source->end (),
					  dest->local () - source->local (),
					  source->rank (),
					  source->displ()+(dest->begin()-source->begin())
					  );
		buffers[dest->rank ()].push_back (d);
		++source;
	      }
	  else
	    ++source;
	else
	  if (source->begin () < dest->end ())
	    if (source->end () < dest->end ())
	      {
		SpatialNegotiationData d (source->begin (),
					  source->end (),
					  dest->local () - source->local (),
					  source->rank (),
					  source->displ()
					  );
		buffers[dest->rank ()].push_back (d);
		++source;
	      }
	    else
	      {
		SpatialNegotiationData d (source->begin (),
					  dest->end (),
					  dest->local () - source->local (),
					  source->rank (),
					  source->displ()
					  );
		buffers[dest->rank ()].push_back (d);
		++dest;
	      }
	  else
	    ++dest;
      }
  }


  void
  SpatialNegotiator::send (MPI_Comm& comm,
			   int destRank,
			   NegotiationIntervals& intervals)
  {
    SpatialNegotiationData* data = &intervals[0];
    int nIntervals = intervals.size ();
    // first send size
    MPI_Send (&nIntervals, 1, MPI_INT, destRank, SPATIAL_NEGOTIATION_MSG, comm);
    MPI_Send (data,
	      sizeof (SpatialNegotiationData) / sizeof (int) * nIntervals,
	      MPI_INT,
	      destRank,
	      SPATIAL_NEGOTIATION_MSG,
	      comm);
  }


  void
  SpatialNegotiator::receive (MPI_Comm& comm,
			      int sourceRank,
			      NegotiationIntervals& intervals)
  {
    int nIntervals;
    MPI_Recv (&nIntervals, 1, MPI_INT, sourceRank, SPATIAL_NEGOTIATION_MSG, comm,
	      MPI_STATUS_IGNORE);
    intervals.resize (nIntervals);
    MPI_Recv (&intervals[0],
	      sizeof (SpatialNegotiationData) / sizeof (int)
	      * nIntervals,
	      MPI_INT,
	      sourceRank,
	      SPATIAL_NEGOTIATION_MSG,
	      comm,
	      MPI_STATUS_IGNORE);
  }


  void
  SpatialNegotiator::allToAll (std::vector<NegotiationIntervals>& out,
			       std::vector<NegotiationIntervals>& in)
  {
    if (out.size () != nProcesses || in.size () != nProcesses)
      error ("internal error in SpatialNegotiator::allToAll ()");
    in[localRank] = out[localRank];
    for (unsigned int i = 0; i < localRank; ++i)
      receive (comm, i, in[i]);
    for (unsigned int i = localRank + 1; i < nProcesses; ++i)
      send (comm, i, out[i]);
    for (unsigned int i = 0; i < localRank; ++i)
      send (comm, i, out[i]);
    for (unsigned int i = localRank + 1; i < nProcesses; ++i)
      receive (comm, i, in[i]);
  }
  NegotiationIterator
  SpatialNegotiator::negotiateSimple()
  {
      return wrapIntervals (indices->begin (),
			    indices->end (),
			    type,
			    localRank);
  }

  SpatialOutputNegotiator::SpatialOutputNegotiator (IndexMap* indices,
						    Index::Type type, MPI_Comm c,
						    MPI_Comm ic)
      : SpatialNegotiator (indices, type,  c)
  {
	     intercomm = ic;
	    negotiateWidth ();
  }


  NegotiationIterator
  SpatialOutputNegotiator::negotiate (int remoteNProc,
		    Connector* connector)
  {
#ifdef MUSIC_DEBUG
	  connector_ = connector;
#endif
	local.resize (nProcesses);
	results.resize (nProcesses);
	remote.resize (remoteNProc);
    NegotiationIterator mappedDist = wrapIntervals (indices->begin (),
						    indices->end (),
						    type,
						    localRank);
    NegotiationIterator canonicalDist
      = canonicalDistribution (width, nProcesses);

    // NOTE: Find a better name for variable `results'
    intersectToBuffers (mappedDist, canonicalDist, results);

    // Send to virtual connector
    allToAll (results, local);

    // Receive from remote connector
    for (int i = 0; i < remoteNProc; ++i)
      receive (intercomm, i, remote[i]);
    
    results.resize (remoteNProc);
    intersectToBuffers (local, remote, results);

    // Send to remote connector
    for (int i = 0; i < remoteNProc; ++i)
      send (intercomm, i, results[i]);
    
    results.resize (nProcesses);
    intersectToBuffers (remote, local, results);

    // Send back to real connector
    allToAll (results, local);

    return NegotiationIterator (local);
  }
  
  SpatialInputNegotiator::SpatialInputNegotiator (IndexMap* indices,
						    Index::Type type, MPI_Comm c,
						    MPI_Comm ic)
      : SpatialNegotiator (indices, type,  c)
  {
	  intercomm = ic;
	    negotiateWidth ();
  }


  NegotiationIterator
  SpatialInputNegotiator::negotiate ( int remoteNProc,
		    Connector* connector)
  {
#ifdef MUSIC_DEBUG
	  connector_ = connector;
#endif
    remote.resize (remoteNProc);
    NegotiationIterator mappedDist = wrapIntervals (indices->begin (),
						    indices->end (),
						    type,
						    localRank);
    NegotiationIterator canonicalDist
      = canonicalDistribution (width, remoteNProc);
    intersectToBuffers (mappedDist, canonicalDist, remote);
    for (int i = 0; i < remoteNProc; ++i)
      send (intercomm, i, remote[i]);
    for (int i = 0; i < remoteNProc; ++i)
      receive (intercomm, i, remote[i]);
    
    return NegotiationIterator (remote);
  }

}
#endif
