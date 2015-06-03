/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2009 INCF
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

#include "music/index_map_factory.hh"
#include <algorithm>
namespace MUSIC {
  
  IndexMapFactory::IndexMapFactory ()
  {
  }
  

  IndexMapFactory::IndexMapFactory (std::vector<IndexInterval>& indices)
    : indices_ (indices)
  {
  }


  void
  IndexMapFactory::add (int begin, int end, int local)
  {
    indices_.push_back (IndexInterval (begin, end, begin - local));
  }
  

  void
  IndexMapFactory::build ()
  {
    sort (indices_.begin (), indices_.end ());
  }
  

  IndexMap::iterator
  IndexMapFactory::begin ()
  {
    return IndexMap::iterator (new iterator (&indices_.front ()));
  }

  
  const IndexMap::iterator
  IndexMapFactory::end () const
  {
    return IndexMap::iterator (new iterator (&indices_.back () + 1));
  }


  IndexMap*
  IndexMapFactory::copy ()
  {
    return new IndexMapFactory (indices_);
  }

}
