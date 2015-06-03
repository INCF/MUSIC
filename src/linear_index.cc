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

#include "music/linear_index.hh"

namespace MUSIC {
  
  LinearIndex::iterator::iterator (LinearIndex* li)
    : indices_ (li)
  {
  }


  const IndexInterval
  LinearIndex::iterator::operator* ()
  {
    return indices_->interval_;
  }


  const IndexInterval*
  LinearIndex::iterator::dereference ()
  {
    return &indices_->interval_;
  }


  void
  LinearIndex::iterator::operator++ ()
  {
    indices_ = 0;
  }


  bool
  LinearIndex::iterator::isEqual (IteratorImplementation* i) const
  {
    return indices_ == static_cast<iterator*> (i)->indices_;
  }
  
  
  LinearIndex::LinearIndex (GlobalIndex baseindex, int size)
    : interval_ (baseindex, baseindex + size, static_cast<int> (baseindex))
  {

  }


  IndexMap::iterator
  LinearIndex::begin ()
  {
    return IndexMap::iterator (new iterator (this));
  }

  
  const IndexMap::iterator
  LinearIndex::end () const
  {
    return IndexMap::iterator (new iterator (0));
  }


  IndexMap*
  LinearIndex::copy ()
  {
    return new LinearIndex (*this);
  }
  
}
