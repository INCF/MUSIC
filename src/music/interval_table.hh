/*
 *  This file is distributed together with MUSIC.
 *  Copyright (C) 2008, 2009 Mikael Djurfeldt
 *
 *  This interval table implementation is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software
 *  Foundation; either version 3 of the License, or (at your option)
 *  any later version.
 *
 *  The interval table implementation is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MUSIC_INTERVAL_TABLE_HH

/*
 * The interval table implementation can be adjusted by defining the
 * preprocessor macro MUSIC_ITABLE_FLAVOR to one of the following:
 */

#define MUSIC_VANILLA   0 // intervals stored in a linked list in one
			  // contiguous memory block

#define MUSIC_TRIMMED   1 // linked list block trimmed during build

#define MUSIC_COMPACT   2 // linked list block freed during build and
			  // replaced by a single array of intervals

#define MUSIC_SCATTERED 3 // list block freed during build and
			  // replaced by many small arrays of
			  // intervals---one per key in the table

#ifndef MUSIC_ITABLE_FLAVOR
#define MUSIC_ITABLE_FLAVOR MUSIC_VANILLA
#endif

//#define MUSIC_ITABLE_COMPRESSED

#include <vector>
#include <map>
#include <limits>
#include <algorithm>
//#include <hash_map>

extern "C" {
#include <assert.h>
}

#include "ordered_ilist.hh"

namespace MUSIC {

  template<class PointType, class IntervalType, class DataType>
  class IntervalTable {
  private:
    typedef unsigned short DataIndex;
    static const DataIndex MAXDATA = 65535;

    typedef OrderedIList<DataIndex> IList;

    class PreEntry {
      IList list_;
      IList hint_;
    public:
      PreEntry () : list_ (IList::NIL), hint_ (list_) { }
      IList list () { return list_; }
      void insert (DataIndex i) { hint_ = list_.insert (i, hint_); }
    };

    // was hash_map
    typedef std::map<PointType, PreEntry> PreEntryMap;
    PreEntryMap* preEntryMap_;

    class Entry {
#ifdef MUSIC_ITABLE_COMPRESSED
      PointType key_;
#endif
#if MUSIC_ITABLE_FLAVOR == MUSIC_VANILLA || MUSIC_ITABLE_FLAVOR == MUSIC_TRIMMED
      IList ls_;
    public:
#ifdef MUSIC_ITABLE_COMPRESSED
      Entry (PointType key) : key_ (key), ls_ (IList::NIL) { }
      Entry (PointType key, IList ls) : key_ (key), ls_ (ls) { }
#else
      Entry () : ls_ (IList::NIL) { }
      Entry (IList ls) : ls_ (ls) { }
#endif
      IList list () { return ls_; }
#else
      DataIndex n_;
      IList::Interval* ivals_;
    public:
#ifdef MUSIC_ITABLE_COMPRESSED
      Entry (PointType key)
	: key_ (key), n_ (0), ivals_ (NULL) { }
      Entry (PointType key, IList::Interval* ivals, int n)
	: key_ (key), n_ (n), ivals_ (ivals) { }
#else
      Entry ()
	: n_ (0), ivals_ (NULL) { }
      Entry (IList::Interval* ivals, int n)
	: n_ (n), ivals_ (ivals) { }
#endif
      IList::Interval* intervals () const { return ivals_; }
      int nIntervals () const { return n_; }
#endif
#ifdef MUSIC_ITABLE_COMPRESSED
      PointType key () { return key_; }
#endif
    };

    std::vector<Entry> entryTable_;
    std::vector<DataType> data_;
#if MUSIC_ITABLE_FLAVOR == MUSIC_COMPACT
    IList::Interval* iArray;
#endif

    class SortCriterion {
      // We use this ugly hack in order to be able to use the stl::map
      // for looking up Data indices. The more proper approach is to
      // include a hash map implementation in MUSIC. We might use one
      // of Googles sparse_map implementations.
      struct Mem0 {
	int f0;
      };
      struct Mem1 {
	void* f0;
      };
      struct Mem2 {
	int f0;
	void* f1;
      };
    public:
      bool operator() (const DataType& x, const DataType& y) const
      {
	const DataType* px = &x;
	const DataType* py = &y;
	// the compiler optimization will resolve this at compile time
	if (sizeof (DataType) == sizeof (Mem0))
	  {
	    const Mem0 u = *reinterpret_cast<const Mem0*> (px);
	    const Mem0 v = *reinterpret_cast<const Mem0*> (py);
	    return u.f0 < v.f0;
	  }
	else if (sizeof (DataType) == sizeof (Mem1))
	  {
	    const Mem1 u = *reinterpret_cast<const Mem1*> (px);
	    const Mem1 v = *reinterpret_cast<const Mem1*> (py);
	    return u.f0 < v.f0;
	  }
	else if (sizeof (DataType) == sizeof (Mem2))
	  {
	    const Mem2 u = *reinterpret_cast<const Mem2*> (px);
	    const Mem2 v = *reinterpret_cast<const Mem2*> (py);
	    return u.f1 < v.f1 || (u.f1 == v.f1 && u.f0 < v.f0);
	  }
	else
	  {
	    // report error
	    assert (sizeof (DataType) == sizeof (Mem0));
	    return false; // satisfy compiler
	  }
      }
    };

    typedef std::map<DataType, DataIndex, SortCriterion> DataMap;
    DataMap* dataMap_;

    PointType lowerBound_;
    PointType rangeSize_;
    unsigned int tableSize_;
    void recordProperties (const IntervalType& ival);

    unsigned int nIntervals_;

  public:
    class Action {
    public:
      virtual void operator() (DataType& data) = 0;
    };

    IntervalTable ()
      : lowerBound_ (std::numeric_limits<PointType>::max ()),
	rangeSize_ (std::numeric_limits<PointType>::min ()),
	tableSize_ (0),
	nIntervals_ (0)
    {
      preEntryMap_ = new PreEntryMap ();
      dataMap_ = new DataMap ();
    }
    ~IntervalTable ()
    {
#if MUSIC_ITABLE_FLAVOR == MUSIC_COMPACT
      delete[] iArray;
#elif MUSIC_ITABLE_FLAVOR == MUSIC_SCATTERED
      for (typename std::vector<Entry>::iterator i = entryTable_.begin ();
	   i != entryTable_.end ();
	   ++i)
	delete[] i->intervals();
#endif
    }
    void add (const IntervalType& i, const DataType& data);
    void build ();
    void search (PointType point, Action* a);
    int size () const { return nIntervals_; }
    int tableSize () const { return entryTable_.size (); }
  };

  template<class PointType, class IntervalType, class DataType>
  void
  IntervalTable<PointType, IntervalType, DataType>::recordProperties
  (const IntervalType& ival)
  {
    lowerBound_ = std::min (lowerBound_, ival.begin ());
    rangeSize_ = std::max (rangeSize_, ival.end ());
    nIntervals_ += 1;
  }

  template<class PointType, class IntervalType, class DataType>
  void
  IntervalTable<PointType, IntervalType, DataType>::add (const IntervalType& ival,
							 const DataType& data)
  {
    // assign a DataIndex to data
    typename DataMap::iterator pos = dataMap_->find (data);
    DataIndex dataIndex;
    if (pos == dataMap_->end ())
      {
	dataIndex = data_.size ();
	assert (dataIndex < MAXDATA);
	data_.push_back (data);
	(*dataMap_)[data] = dataIndex;
      }
    else
      dataIndex = pos->second;

    recordProperties (ival);

    for (PointType k = ival.begin (); k < ival.end (); ++k)
      (*preEntryMap_)[k].insert (dataIndex);
  }


  template<class PointType, class IntervalType, class DataType>
  void
  IntervalTable<PointType, IntervalType, DataType>::build ()
  {
    delete dataMap_;
    // trim data_;
    std::vector<DataType> (data_).swap (data_);
    tableSize_ = preEntryMap_->size ();
    if (rangeSize_ > 0)
      rangeSize_ = rangeSize_ - lowerBound_;
    else
      rangeSize_ = 0; // no intervals
#ifdef MUSIC_ITABLE_COMPRESSED
    if (rangeSize_ == 0)
      rangeSize_ = 1;
    entryTable_.reserve (tableSize_ + 2);
#else
    entryTable_.resize (rangeSize_);
#endif
#if MUSIC_ITABLE_FLAVOR == MUSIC_COMPACT || MUSIC_ITABLE_FLAVOR == MUSIC_SCATTERED
    IList::Interval* iPtr;
#if MUSIC_ITABLE_FLAVOR == MUSIC_COMPACT
    iArray = new IList::Interval[IList::nNodes ()];
    iPtr = iArray;
#endif
#endif
#ifdef MUSIC_ITABLE_COMPRESSED
    // backward sentinel
    entryTable_.push_back (Entry (std::numeric_limits<PointType>::min ()));
#endif
    for (typename PreEntryMap::iterator p = preEntryMap_->begin ();
	 p != preEntryMap_->end ();
	 ++p)
      {
	PointType key = p->first;
	IList list = p->second.list ();
#if MUSIC_ITABLE_FLAVOR == MUSIC_VANILLA || MUSIC_ITABLE_FLAVOR == MUSIC_TRIMMED
#ifdef MUSIC_ITABLE_COMPRESSED
	entryTable_.push_back (Entry (key, list));
#else
	entryTable_[key - lowerBound_] = Entry (list);
#endif
#else // MUSIC_COMPACT or MUSIC_SCATTERED
	int n = list.size ();
#if MUSIC_ITABLE_FLAVOR == MUSIC_SCATTERED
	iPtr = new IList::Interval[n];
#endif
#ifdef MUSIC_ITABLE_COMPRESSED
	entryTable_.push_back (Entry (key, iPtr, n));
#else
	entryTable_[key - lowerBound_] = Entry (iPtr, n);
#endif
	for (IList::iterator j = list.begin (); j != list.end (); ++j)
	  *iPtr++ = *j;
#endif
      }
#ifdef MUSIC_ITABLE_COMPRESSED
    // forward sentinel
    entryTable_.push_back (Entry (std::numeric_limits<PointType>::max ()));
#endif
    delete preEntryMap_;
#if MUSIC_ITABLE_FLAVOR == MUSIC_TRIMMED
    IList::trimNodes ();
#elif MUSIC_ITABLE_FLAVOR == MUSIC_COMPACT || MUSIC_ITABLE_FLAVOR == MUSIC_SCATTERED
    IList::reset ();
#endif

#ifdef MUSIC_DEBUG
    std::cout << "data: " << data_.size () << std::endl
	      << "nodes: " << IList::nNodes () << std::endl
	      << "freelist: " << IList::freeList ().size ()
	      << std::endl;
#endif
  }
  
  
  template<class PointType, class IntervalType, class DataType>
  void
  IntervalTable<PointType, IntervalType, DataType>::search (PointType p, Action* a)
  {
#ifndef MUSIC_ITABLE_COMPRESSED
    PointType i = p - lowerBound_;
    if (i < 0 || i >= rangeSize_)
      return;
#else
    // obtain approximate position
    PointType i = ((p - lowerBound_) * tableSize_) / rangeSize_ + 1;

    // sentinels are at 0 and tableSize_ + 1
    if (i < 1 || i > PointType (tableSize_))
      return;

    if (p != entryTable_[i].key ())
      {
	// search forward
	if (p > entryTable_[i].key ())
	  {
	    do
	      ++i;
	    // end of table guarded by sentinel
	    while (p > entryTable_[i].key ());
	    if (p != entryTable_[i].key ())
	      return;
	  }
	else
	  // search backward
	  {
	    do
	      --i;
	    // beginning of table guarded by sentinel
	    while (p < entryTable_[i].key ());
	    if (p != entryTable_[i].key ())
	      return;
	  }
      }
#endif

#if MUSIC_ITABLE_FLAVOR == MUSIC_VANILLA || MUSIC_ITABLE_FLAVOR == MUSIC_TRIMMED
    IList ls = entryTable_[i].list ();
    for (IList::iterator j = ls.begin (); j != ls.end (); ++j)
      for (int k = j->begin (); k <= j->end (); ++k)
	(*a) (data_[k]);
#else // MUSIC_COMPACT or MUSIC_SCATTERED
    IList::Interval* ivals = entryTable_[i].intervals ();
    for (int j = 0; j < entryTable_[i].nIntervals (); ++j)
      for (int k = ivals[j].begin (); k <= ivals[j].end (); ++k)
	(*a) (data_[k]);
#endif
  }

}

#define MUSIC_INTERVAL_TABLE_HH
#endif
