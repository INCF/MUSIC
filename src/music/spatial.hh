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

#ifndef MUSIC_NEGOTIATOR_HH
#include "music/music-config.hh"
#if MUSIC_USE_MPI
#include <mpi.h>
#include <vector>
#include <memory>

#include <music/index_map.hh>

namespace MUSIC {

  class SpatialNegotiationData {
    IndexInterval interval_;
    int rank_;
    int displ_;
  public:
    SpatialNegotiationData () { }
    SpatialNegotiationData (IndexInterval i, int r, int displ)
      : interval_ (i), rank_ (r), displ_(displ) { }
    SpatialNegotiationData (int b, int e, int l, int r, int displ)
      : interval_ (b, e, l), rank_ (r), displ_(displ) { }
    const IndexInterval& interval () const { return interval_; }
    int begin () const { return interval_.begin (); }
    int end () const { return interval_.end (); }
    void setEnd (int e) { interval_.setEnd (e); }
    int local () const { return interval_.local (); }
    void setLocal (int l) { interval_.setLocal (l); }
    void setDispl(int displ) {displ_ = displ;}
    int rank () const { return rank_; }
    int displ() const {return displ_; }
  };


  typedef std::vector<SpatialNegotiationData> NegotiationIntervals;
  

  class NegotiationIterator {
  public:
    class Implementation {
    public:
      virtual ~Implementation () { };
      virtual bool end () = 0;
      virtual void operator++ () = 0;
      virtual SpatialNegotiationData* dereference () = 0;
      virtual Implementation* copy () = 0;
    };

    class IntervalTraversal : public Implementation {
      NegotiationIntervals& buffer;
      unsigned int interval;
    public:
      IntervalTraversal (NegotiationIntervals& buffer_)
	: buffer (buffer_), interval (0) { }
      bool end () { return interval == buffer.size (); }
      void operator++ () { ++interval; }
      SpatialNegotiationData* dereference () { return &buffer[interval]; }
      virtual Implementation* copy () { return new IntervalTraversal (*this); }
    };

    class BufferTraversal : public Implementation {
      std::vector<NegotiationIntervals>& buffers;
      unsigned int buffer;
      unsigned int interval;
      void findInterval ();
    public:
      BufferTraversal (std::vector<NegotiationIntervals>& buffers);
      bool end ();
      void operator++ ();
      SpatialNegotiationData* dereference ();
      virtual Implementation* copy () { return new BufferTraversal (*this); }
    };

  private:
    Implementation* implementation_;
    SpatialNegotiationData current_;
    bool end_;
  public:
    NegotiationIterator (Implementation* impl)
    { init (impl); }
    NegotiationIterator (NegotiationIntervals& buffer)
    { init (new IntervalTraversal (buffer)); }
    NegotiationIterator (std::vector<NegotiationIntervals>& buffers)
    { init (new BufferTraversal (buffers)); }
    void init (Implementation* impl);
    ~NegotiationIterator ();
    NegotiationIterator (const NegotiationIterator& i);
    const NegotiationIterator& operator= (const NegotiationIterator& i);
    bool end () { return end_; }
    NegotiationIterator& operator++ ();
    SpatialNegotiationData* operator-> ();
  };

  class Connector;

  // The SpatialNegotiator negotiates with the remote application how
  // to redistribute data over a port pair.  This is done using the
  // algorithm described in Djurfeldt and Ekeberg (2009) with memory
  // complexity O (N / P) where N is the port width and P is the
  // number of MPI processes.

  class SpatialNegotiator {
  protected:
    MPI_Comm comm;
    MPI_Comm intercomm;
    IndexMap* indices;
    Index::Type type;
    std::vector<NegotiationIntervals> remote;
    int width;
    int maxLocalWidth_;
    unsigned int localRank;
    unsigned int nProcesses;
    Connector* connector_; // used only for debugging
  public:
    SpatialNegotiator (IndexMap* indices,
    		Index::Type type,
    		MPI_Comm c);
    virtual ~SpatialNegotiator (){};

    int getWidth(){return width;}
    int maxLocalWidth () { return maxLocalWidth_; }
    NegotiationIterator wrapIntervals (IndexMap::iterator beg,
				       IndexMap::iterator end,
				       Index::Type type,
				       int rank);
    void send (MPI_Comm& comm, int destRank,
	       NegotiationIntervals& intervals);
    void receive (MPI_Comm& comm, int sourceRank,
		  NegotiationIntervals& intervals);
    void allToAll (std::vector<NegotiationIntervals>& out,
		   std::vector<NegotiationIntervals>& in);
    NegotiationIterator canonicalDistribution (int width, int nProcesses);
    void intersectToBuffers (std::vector<NegotiationIntervals>& source,
			     std::vector<NegotiationIntervals>& dest,
			     std::vector<NegotiationIntervals>& buffers);
    void intersectToBuffers (NegotiationIterator source,
			     NegotiationIterator dest,
			     std::vector<NegotiationIntervals>& buffers);
  private:
    void negotiateWidth ();
    void intersectToBuffers2 (NegotiationIterator source,
			      NegotiationIterator dest,
			      std::vector<NegotiationIntervals>& buffers);
  public:
    virtual NegotiationIterator negotiate ( int remoteNProc, Connector* connector) = 0;
    virtual NegotiationIterator negotiateSimple();
  };


  class SpatialOutputNegotiator : public SpatialNegotiator {
    std::vector<NegotiationIntervals> local;
    std::vector<NegotiationIntervals> results;
  public:
    SpatialOutputNegotiator (IndexMap* indices,
		    Index::Type type, MPI_Comm c,
		    MPI_Comm intercomm);
    NegotiationIterator negotiate ( int remoteNProc,  Connector* connector);
  private:
    void negotiateWidth ();

  };

  
  class SpatialInputNegotiator : public SpatialNegotiator {
  public:
    SpatialInputNegotiator (IndexMap* indices,
		    Index::Type type, MPI_Comm c,
		    MPI_Comm intercomm);
    NegotiationIterator negotiate (int remoteNProc,  Connector* connector);
  private:
    void negotiateWidth ();

  };

}
#endif
#define MUSIC_NEGOTIATOR_HH
#endif
