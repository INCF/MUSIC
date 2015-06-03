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

#ifndef MUSIC_ARRAY_DATA_HH
#include "music/music-config.hh"
#if MUSIC_USE_MPI
#include "music/data_map.hh"

namespace MUSIC {

  /*
   * This data map is part of the MUSIC API and documented
   * in section 4.3.9 of the MUSIC manual.
   */

  class ArrayData : public DataMap {
    MPI::Datatype type_;
    IndexMap* indexMap_;
  public:
    ArrayData (void* buffer, MPI::Datatype type, IndexMap* map);
    ArrayData (void* buffer, MPI::Datatype type, int baseIndex, int size);
    virtual ~ArrayData ();
    virtual DataMap* copy ();
    virtual MPI::Datatype type () { return type_; }
    virtual IndexMap* indexMap () { return indexMap_; }
  };

}
#endif
#define MUSIC_ARRAY_DATA_HH
#endif
