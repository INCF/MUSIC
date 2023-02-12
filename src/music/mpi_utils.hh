/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2022 Mikael Djurfeldt
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

#ifndef MUSIC_MPI_UTILS_HH
#define MUSIC_MPI_UTILS_HH
#include "music/music-config.hh"
#if MUSIC_USE_MPI
#include <mpi.h>

namespace MUSIC
{
  bool mpi_is_initialized ();
  int mpi_get_rank (MPI_Comm comm);
  int mpi_get_comm_size (MPI_Comm comm);
  int mpi_get_group_size (MPI_Group group);
  int mpi_get_type_size (MPI_Datatype type);
  MPI_Group mpi_get_group (MPI_Comm comm);
}

#endif /* MUSIC_USE_MPI */
#endif /* MUSIC_MPI_UTILS_HH */
