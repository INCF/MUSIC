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

#ifndef MUSIC_DATA_MAP_HH
#include "music/music-config.hh"
#if MUSIC_USE_MPI
#include <mpi.h>

#include "music/index_map.hh"

namespace MUSIC {

  typedef char ContDataT;

  /*
   * The current interface should maybe be changed so that data maps
   * are read as a set of pairs of intervals and addresses similar to
   * the EventRoutingData data structure.  This allows for easier
   * extension to more generic types of data maps.
   */

  class DataMap {
  protected:
    void* base_;
  public:
    DataMap () { };
    virtual ~DataMap () { };
    DataMap (void* base) : base_ (base) { };
    virtual DataMap* copy () = 0;
    void* base () const { return base_; }
    virtual MPI_Datatype type () = 0;
    virtual IndexMap* indexMap () = 0;
  };

}
#endif
#define MUSIC_DATA_MAP_HH
#endif
