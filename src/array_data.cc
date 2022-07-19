/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007, 2008, 2009, 2022 INCF
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

// This header file needs to be included first since array_data.hh
// includes <mpi.h> which musy be included first on BG/L
#include "music/array_data.hh"
#if MUSIC_USE_MPI

#include "music/linear_index.hh"

namespace MUSIC {
  
  ArrayData::ArrayData (void* buffer, MPI_Datatype type, IndexMap* map)
    : DataMap (buffer), type_ (type), indexMap_ (map->copy ())
  {
  }

  ArrayData::~ArrayData ()
  {
    delete indexMap_;
  }
  
  ArrayData::ArrayData (void* buffer,
			MPI_Datatype type,
			int baseIndex,
			int size)
    : DataMap (buffer)
  {
    type_ = type;
    indexMap_ = new LinearIndex (baseIndex, size);
  }

  DataMap*
  ArrayData::copy ()
  {
    return new ArrayData (base (), type_, indexMap_);
  }
  
}
#endif
