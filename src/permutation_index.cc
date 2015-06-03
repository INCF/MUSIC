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
#include "music/permutation_index.hh"
#include <algorithm>
namespace MUSIC {
  
  PermutationIndex::PermutationIndex (GlobalIndex* indices, int size)
  {
    // NOTE: collapse adjacent intervals where possible?
    for (int i = 0; i < size; ++i)
      indices_.push_back (IndexInterval (indices[i],
					 indices[i] + 1,
					 indices[i] - i));
    sort (indices_.begin (), indices_.end ());
  }
  

  PermutationIndex::PermutationIndex (std::vector<IndexInterval>& indices)
    : indices_ (indices)
  {
  }

  
  IndexMap::iterator
  PermutationIndex::begin ()
  {
    return IndexMap::iterator (new iterator (&indices_.front ()));
  }

  
  const IndexMap::iterator
  PermutationIndex::end () const
  {
    return IndexMap::iterator (new iterator (&indices_.back () + 1));
  }


  IndexMap*
  PermutationIndex::copy ()
  {
    return new PermutationIndex (indices_);
  }

}
