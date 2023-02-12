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

#include "music/music-config.hh"

#if MUSIC_USE_MPI
#include <mpi.h>
#endif

#include "music/mpi_utils.hh"
#include "music/error.hh"

#include <iostream>
#include <stdlib.h>

namespace MUSIC {

  void
  error ()
  {
#if MUSIC_USE_MPI
    MPI_Abort (MPI_COMM_WORLD, 1);
#else
    abort();
#endif
  }


  void
  hang ()
  {
    while (true)
      ;
  }
  
  
  void
  error (std::string msg)
  {
    std::cerr << "Error in MUSIC library: " << msg << std::endl;
    error ();
  }


  void
  error (std::ostringstream& ostr)
  {
    error (ostr.str ());
  }


  static int
  getRank ()
  {
#if MUSIC_USE_MPI
    int isInitialized;
    MPI_Initialized (&isInitialized);
    if (isInitialized)
      return mpi_get_rank (MPI_COMM_WORLD);
#endif
      return -1;
  }

  
  void
  error0 (std::string msg)
  {
    if (getRank () <= 0)
      error (msg);
    else
      // Give process #0 a chance to report the error
      hang ();
  }


  void
  errorRank (std::string msg)
  {
    std::ostringstream text;
    int rank = getRank ();
    if (rank >= 0)
      text << "rank #" << rank << ": ";
    text << msg;
    error (text.str ());
  }
  
  
  void
  checkOnce (bool& flag, std::string msg)
  {
    if (flag)
      errorRank (msg);
    flag = true;
  }


  void
  checkInstantiatedOnce (bool& isInstantiated, std::string className)
  {
    std::ostringstream msg;
    msg << className << " constructor was called a second time.\n"
      "Only one " << className << " object can exist at any instance of time.";
    checkOnce (isInstantiated, msg.str ());
  }

  
  void
  checkCalledOnce (bool& isCalled, std::string funcName, std::string suffix)
  {
    std::ostringstream msg;
    msg << funcName << " called twice" << suffix;
    checkOnce (isCalled, msg.str ());
  }
  
}
