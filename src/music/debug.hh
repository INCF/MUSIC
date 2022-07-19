/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2008, 2009, 2022 INCF
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
#ifndef MUSIC_DEBUG_HH

#ifdef MUSIC_DEBUG

#include <iostream>

#define MUSIC_LOG(X) (std::cerr << X << std::endl << std::flush)
#if MUSIC_USE_MPI
#define MUSIC_LOGN(N, X) \
  { if (mpi_get_rank (MPI_COMM_WORLD) == N) MUSIC_LOG (X); }

#define MUSIC_LOG0(X) MUSIC_LOGN (0, X)

#define MUSIC_LOGR(X)					\
  {							\
    std::cerr << mpi_get_rank (MPI_COMM_WORLD) << ": "	\
              << X << std::endl << std::flush;		\
  }

#define MUSIC_LOGRE(X)					\
  {							\
    int _r = mpi_get_rank (MPI_COMM_WORLD);		\
    char* _e = getenv ("MUSIC_DEBUG_RANK");		\
    if (_e != NULL && atoi (_e) == _r)			\
      {							\
	std::cerr << _r << ": "				\
		  << X << std::endl << std::flush;	\
      }							\
  }

#define MUSIC_LOGBR(C, X)			\
  {						\
    int _r;                                     \
    MPI_Comm_rank ((C), &_r);			\
    int _n;                                     \
    MPI_Comm_size ((C), &_n);			\
    for (int _i = 0; _i < _n; ++_i)		\
      {						\
	MPI_Barrier ((C));			\
	if (_i == _r)				\
	  MUSIC_LOGR (X);			\
      }						\
  }

#define MUSIC_LOGX(X)
#endif

#else

#define MUSIC_LOG(X)
#define MUSIC_LOGN(N, X)
#define MUSIC_LOG0(X)
#define MUSIC_LOGR(X)
#define MUSIC_LOGRE(X)
#define MUSIC_LOGBR(C, X)
#define MUSIC_LOGX(X)

#endif

#define MUSIC_DEBUG_HH
#endif
