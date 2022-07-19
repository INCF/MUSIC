/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2012, 2022 INCF
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

#include <music.hh>

#include <iostream>

int
main (int argc, char *argv[])
{
  MUSIC::Setup* musicSetup = new MUSIC::Setup (argc, argv);
  MPI_Comm comm = musicSetup->communicator ();
  int rank = MUSIC::mpi_get_rank (comm);
  double param;
  if (!musicSetup->config ("param", &param))
    param = -1.0;
  std::cout << "rank=" << rank << ":param=" << param;
  for (int i = 0; i < argc; ++i)
    std::cout << ':' << argv[i];
  std::cout << std::endl;
  MPI_Finalize ();
  return 0;
}
