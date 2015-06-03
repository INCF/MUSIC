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

#ifndef MUSIC_COMMUNICATION_HH

#include "music/data_map.hh"

namespace MUSIC {

  // These are the MPI message tags used by the MUSIC library
  
  enum MessageTag {
    CREATE_INTERCOMM_MSG,
    TICKINTERVAL_MSG,
    WIDTH_MSG,
    SPATIAL_NEGOTIATION_MSG,
    CONT_MSG,
    SPIKE_MSG,
    MESSAGE_MSG,
    FLUSH_MSG
  };

}

#define MUSIC_COMMUNICATION_HH
#endif
