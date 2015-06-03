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

#ifndef MUSIC_INDEX_MAP_HH
#include <memory>

#include <music/interval.hh>

namespace MUSIC {

  class Index {
  public:
    enum Type { GLOBAL, LOCAL, UNDEFINED };
    static int WILDCARD_MAX;
  };

  class GlobalIndex : public Index {
    int id;
  public:
    GlobalIndex () { }
    GlobalIndex (int i) : id (i) { }
    operator int () const { return id; }
  };

  class LocalIndex : public Index {
    int id;
  public:
    LocalIndex () { }
    LocalIndex (int i) : id (i) { }
    operator int () const { return id; }
  };

  class IndexInterval : public Interval {
    LocalIndex local_;
  public:
    IndexInterval () { }
    IndexInterval (GlobalIndex b, GlobalIndex e, LocalIndex l)
      : Interval (b, e), local_ (l) { }
    LocalIndex local () const { return local_; }
    void setLocal (int l) { local_ = l; }
  };

  bool operator< (const IndexInterval& a, const IndexInterval& b);

  class IndexMap {
  public:
    class IteratorImplementation {
    public:
      virtual ~IteratorImplementation () { }
      virtual const IndexInterval operator* () = 0;
      virtual const IndexInterval* dereference () = 0;
      virtual bool isEqual (IteratorImplementation* i) const = 0;
      virtual void operator++ () = 0;
      virtual IteratorImplementation* copy () = 0;
    };

    
    class iterator {
      IteratorImplementation* implementation_;
    public:
      iterator (IteratorImplementation* impl)
	: implementation_ (impl) { }
      ~iterator ()
      {
	delete implementation_;
      }
      iterator (const iterator& i)
	: implementation_ (i.implementation_->copy ())
      {
      }
      const iterator& operator= (const iterator& i)
      {
	delete implementation_;
	implementation_ = i.implementation_->copy ();
	return *this;
      }
      IteratorImplementation* implementation () const
      {
	return implementation_;
      }
      const IndexInterval operator* ();
      const IndexInterval* operator-> ();
      bool operator== (const iterator& i) const;
      bool operator!= (const iterator& i) const;
      iterator& operator++ ();
    };
    

    virtual ~IndexMap () { }
    virtual iterator begin () = 0;
    virtual const iterator end () const = 0;

    virtual IndexMap* copy () = 0;
  };

}
#define MUSIC_INDEX_MAP_HH
#endif
