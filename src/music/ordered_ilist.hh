/*
 *  This file is distributed together with MUSIC.
 *  Copyright (C) 2012 Mikael Djurfeldt
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

#ifndef MUSIC_ORDERED_ILIST_HH

#include <vector>

extern "C" {
#include <assert.h>
}

namespace MUSIC {

  template<class DataType>
  class OrderedIList {
    typedef int ListPtr;

    static const ListPtr NILPTR = -1;

  public:
    class Interval {
      DataType begin_;
      DataType end_;
    public:
      Interval () { }
      Interval (DataType b, DataType e) : begin_ (b), end_ (e) { }
      DataType begin () const { return begin_; }
      void setBegin (const DataType x) { begin_ = x; }
      DataType end () const { return end_; }
      void setEnd (const DataType x) { end_ = x; }
    };

  private:
    struct Node {
      Node (DataType b, DataType e, ListPtr n)
	: ival_ (Interval (b, e)), next_ (n)
      { }
      Interval ival_; // inclusive
      ListPtr next_;
    };

    static std::vector<Node> node_;
    static OrderedIList freePtr_;

    ListPtr lptr_;

    DataType begin_ () const { return node_[lptr_].ival_.begin (); }
    void setBegin_ (const DataType x) { node_[lptr_].ival_.setBegin (x); }
    DataType end_ () const { return node_[lptr_].ival_.end (); }
    void setEnd_ (const DataType x) { node_[lptr_].ival_.setEnd (x); }
    OrderedIList next_ () const { return OrderedIList (node_[lptr_].next_); }
    void setNext_ (const OrderedIList list) const { node_[lptr_].next_ = list.lptr_; }
    OrderedIList cons (const DataType b, const DataType e, const OrderedIList n);
    void free (const OrderedIList list);
  public:
    static const OrderedIList NIL;

    class iterator {
      ListPtr lptr_;
      OrderedIList list () const { return OrderedIList (lptr_); }
    public:
      iterator (OrderedIList list) : lptr_ (list.lptr_) { }
      bool operator!= (const iterator& i) const
      {
	return lptr_ != i.lptr_;
      }
      iterator& operator++ () { lptr_ = list ().next_ ().lptr_; return *this; }
      const Interval& operator* () const
      {
	return node_[lptr_].ival_;
      }
      const Interval* operator-> () const
      {
	return &node_[lptr_].ival_;
      }
    };

    OrderedIList () : lptr_ (NILPTR) { }
    OrderedIList (ListPtr ptr) : lptr_ (ptr) { }
    bool operator== (const OrderedIList& list) const { return lptr_ == list.lptr_; }
    bool operator!= (const OrderedIList& list) const { return lptr_ != list.lptr_; }
    bool isEmpty () const { return lptr_ == NILPTR; }
    OrderedIList insert (DataType i) { return insert (i, *this); }
    OrderedIList insert (DataType i, OrderedIList hint);
    iterator begin () const { return iterator (*this); }
    iterator end () const { return iterator (NIL); }
    unsigned int size () const;

    static unsigned int nNodes () { return node_.size (); }
    static void trimNodes () { std::vector<Node> (node_).swap (node_); }
    static void reset () { freePtr_ = NIL; node_.clear (); trimNodes (); }

#ifdef MUSIC_DEBUG
    static OrderedIList freeList () { return freePtr_; }
#endif
  };

  template<class DataType>
  std::vector<typename OrderedIList<DataType>::Node> OrderedIList<DataType>::node_;

  template<class DataType>
  OrderedIList<DataType> OrderedIList<DataType>::freePtr_
  = OrderedIList<DataType>::NIL;

  template<class DataType>
  const OrderedIList<DataType> OrderedIList<DataType>::NIL;

  template<class DataType>
  OrderedIList<DataType>
  OrderedIList<DataType>::cons (const DataType b,
				const DataType e,
				const OrderedIList n)
  {
    if (freePtr_.isEmpty ())
      {
	// create a new node
	ListPtr lptr = node_.size ();
	node_.push_back (Node (b, e, n.lptr_));
	return OrderedIList (lptr);
      }
    else
      {
	// unlink from freelist
	OrderedIList list = freePtr_;
	freePtr_ = freePtr_.next_ ();
	list.setBegin_ (b);
	list.setEnd_ (e);
	list.setNext_ (n);
	return list;
      }
  }

  template<class DataType>
  void
  OrderedIList<DataType>::free (const OrderedIList list)
  {
    list.setNext_ (freePtr_);
    freePtr_ = list;
  }

  template<class DataType>
  OrderedIList<DataType>
  OrderedIList<DataType>::insert (DataType i, OrderedIList hint)
  {
    if (hint.isEmpty ())
      {
	if (isEmpty ())
	  return *this = cons (i, i, NIL);
	if (i < begin_ ())
	  {
	    if (i + 1 == begin_ ())
	      setBegin_ (i);
	    else
	      *this = cons (i, i, *this);
	    return *this;
	  }
	OrderedIList list = *this;
	while (true)
	  {
	    OrderedIList next = list.next_ ();
	    assert (i > list.end_ ());
	    if (next.isEmpty ())
	      {
		if (i == list.end_ () + 1)
		  {
		    list.setEnd_ (i);
		    return list;
		  }
	      }
	    else
	      {
		if (i == list.end_ () + 1)
		  {
		    if (i + 1 == next.begin_ ())
		      {
			list.setEnd_ (next.end_ ());
			list.setNext_ (next.next_ ());
			free (next);
		      }
		    else
		      {
			list.setEnd_ (i);
		      }
		    return list;
		  }
	      }
	    if (next.isEmpty ())
	      {
		next = cons (i, i, NIL);
		list.setNext_ (next);
		return next;
	      }
	    if (i < next.begin_ ())
	      {
		if (i + 1 == next.begin_ ())
		  next.setBegin_ (i);
		else
		  {
		    next = cons (i, i, next);
		    list.setNext_ (next);
		  }
		return next;
	      }
	    list = next;
	  }
      }
    else
      {
	if (i < hint.begin_ () || !hint.next_ ().isEmpty ())
	  return insert (i, NIL);
	assert (i > end_ ());
	if (i == hint.end_ () + 1)
	  {
	    hint.setEnd_ (i);
	    return hint;
	  }
	else
	  {
	    OrderedIList next = cons (i, i, NIL);
	    hint.setNext_ (next);
	    return next;
	  }
      }
  }

  template<class DataType>
  unsigned int
  OrderedIList<DataType>::size () const
  {
    unsigned int size = 0;
    OrderedIList list = *this;
    while (!list.isEmpty ())
      {
	++size;
	list = list.next_ ();
      }
    return size;
  }

}

#define MUSIC_ORDERED_ILIST_HH
#endif // MUSIC_ORDERED_ILIST_HH
