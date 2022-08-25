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

#include "music/mpi_utils.hh"

namespace MUSIC {

  bool
  mpi_is_initialized ()
  {
    int isInitialized;
    MPI_Initialized (&isInitialized);
    return isInitialized;
  }

  int
  mpi_get_rank (MPI_Comm comm)
  {
    int rank;
    MPI_Comm_rank (comm, &rank);
    return rank;
  }

  int
  mpi_get_comm_size (MPI_Comm comm)
  {
    int size;
    MPI_Comm_size (comm, &size);
    return size;
  }

  int
  mpi_get_group_size (MPI_Group group)
  {
    int size;
    MPI_Group_size (group, &size);
    return size;
  }

  int
  mpi_get_type_size (MPI_Datatype type)
  {
    int size;
    MPI_Type_size (type, &size);
    return size;
  }

  MPI_Group
  mpi_get_group (MPI_Comm comm)
  {
    MPI_Group group;
    MPI_Comm_group (comm, &group);
    return group;
  }

}
