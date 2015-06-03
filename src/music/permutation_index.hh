/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2008, 2009 INCF
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

#ifndef MUSIC_PERMUTATION_INDEX_HH
#include <vector>
#include "music/index_map.hh"

namespace MUSIC {

  /*
   * This index map is part of the MUSIC API and documented
   * in section 4.3.8 of the MUSIC manual.
   */

  class PermutationIndex : public IndexMap {
    std::vector<IndexInterval> indices_;
  public:
    class iterator : public IndexMap::IteratorImplementation {
      const IndexInterval* intervalPtr;
    public:
      iterator (const IndexInterval* ptr) : intervalPtr (ptr) { }
      virtual const IndexInterval operator* () { return *intervalPtr; }
      virtual const IndexInterval* dereference () { return intervalPtr; }
      virtual bool isEqual (IteratorImplementation* i) const
      {
	return intervalPtr == static_cast<iterator*> (i)->intervalPtr;
      }
      virtual void operator++ () { ++intervalPtr; }
      virtual IteratorImplementation* copy ()
      {
	return new iterator (intervalPtr);
      }
    };
    
    PermutationIndex (GlobalIndex *indices, int size);
    PermutationIndex (std::vector<IndexInterval>& indices);
    virtual IndexMap::iterator begin ();
    virtual const IndexMap::iterator end () const;
    virtual IndexMap* copy ();    
  };

}
#define MUSIC_PERMUTATION_INDEX_HH
#endif
