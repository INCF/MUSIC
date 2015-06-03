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
#include <limits>

#include "music/index_map.hh"

namespace MUSIC {

  int Index::WILDCARD_MAX = std::numeric_limits<int>::max ();

  bool operator< (const IndexInterval& a, const IndexInterval& b)
  {
    return (a.begin () < b.begin ()
	    || (a.begin () == b.begin () && a.end () < b.end ()));
  }
  
  const IndexInterval
  IndexMap::iterator::operator* ()
  {
    return **implementation_;
  }
  

  const IndexInterval*
  IndexMap::iterator::operator-> ()
  {
    return implementation_->dereference ();
  }
  

  bool
  IndexMap::iterator::operator== (const iterator& i) const
  {
    return implementation_->isEqual (i.implementation ());
  }
  

  bool
  IndexMap::iterator::operator!= (const iterator& i) const
  {
    return !implementation_->isEqual (i.implementation ());
  }
  

  IndexMap::iterator&
  IndexMap::iterator::operator++ ()
  {
    ++*implementation_;
    return *this;
  }

}
