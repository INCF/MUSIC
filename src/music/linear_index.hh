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

#ifndef MUSIC_LINEAR_INDEX_HH
#include "music/index_map.hh"

namespace MUSIC {

  /*
   * This index map is part of the MUSIC API and documented
   * in section 4.3.8 of the MUSIC manual.
   */

  class LinearIndex : public IndexMap {
    IndexInterval interval_;
  public:
    class iterator : public IndexMap::IteratorImplementation {
      LinearIndex* indices_;
    public:
      iterator (LinearIndex* li);
      virtual const IndexInterval operator* ();
      virtual const IndexInterval* dereference ();
      virtual bool isEqual (IteratorImplementation* i) const;
      virtual void operator++ ();
      virtual IteratorImplementation* copy ()
      {
	return new iterator (indices_);
      }
    };

    LinearIndex (GlobalIndex baseindex, int size);
    virtual IndexMap::iterator begin ();
    virtual const IndexMap::iterator end () const;
    virtual IndexMap* copy ();
  };

}
#define MUSIC_LINEAR_INDEX_HH
#endif
