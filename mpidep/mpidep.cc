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

#include "config.h"

#include <string>
#include <cstring>
#include <sstream>
#include <fstream>

extern "C" {
#include <stdlib.h>
}

#include "mpidep.hh"

// Implementation-dependent code

#ifndef CRAY_XE6
#ifdef HAVE_RTS_GET_PERSONALITY
#define BGL
#else
#ifdef HAVE_OMPI_COMM_FREE
#define OPENMPI
#else
#ifdef HAVE_MPICH2
#define MPICH2
#else
#define MPICH
#endif
#endif
#endif
#endif

#ifdef BGL
#include <rts.h>
#endif

#ifdef CRAY_XE6
extern "C" {
#include <sched.h>
}
#endif

/*
 * Predict the local MPI process rank before having called MPI_Init
 *
 * MPI implementations typically passes meta information either through
 * argc and argv (MPICH) or through the environment (OpenMPI).
 *
 * return -1 on failure
 */ 

int
getRank (int argc, char *argv[])
{
#ifdef BGL
  BGLPersonality p;
  rts_get_personality (&p, sizeof (p));
  int pid = rts_get_processor_id ();
  unsigned int rank, np;
  rts_rankForCoordinates (p.xCoord, p.yCoord, p.zCoord, pid, &rank, &np);
  return rank;
#endif
#ifdef MPICH2
  char *pmi_rank = getenv("PMI_RANK");
  int rank;
  if (pmi_rank==NULL)
    return -1;
  else {
    rank = atoi(pmi_rank);
    if (rank<0) return -1;
    return rank;
  }
#endif
#ifdef MPICH
  int seenP4arg = 0;
  int rank;
  const std::string rankopt = "-p4rmrank";
  for (int i = argc - 2; i > 0; --i)
    {
      if (!strncmp (argv[i], "-p4", 3))
	seenP4arg = 1;
      if (argv[i] == rankopt)
	{
	  std::istringstream iss (argv[i + 1]);
	  iss >> rank;
	  return rank;
	}
    }
  return seenP4arg ? 0 : -1;
#endif
#ifdef OPENMPI
  char* vpid = getenv ("OMPI_MCA_ns_nds_vpid");
  if (vpid == NULL)
    vpid = getenv ("OMPI_COMM_WORLD_RANK");
  if (vpid == NULL)
    return -1;
  std::istringstream iss (vpid);
  int rank;
  iss >> rank;
  return rank;
#endif
#ifdef CRAY_XE6
  std::ifstream fnid ("/proc/cray_xt/nid");
  int nid;
  fnid >> nid;
  fnid.close ();
  std::ifstream fnids (getenv ("MUSIC_NODEFILE"));
  int n = 0;
  int i;
  while (fnids)
    {
      fnids >> i;
      if (i == nid)
	{
	  int core = sched_getcpu ();
	  return 24 * n + core;
	}
      ++n;
    }
  return -1;
#endif
}


#ifdef MPICH
std::string
getSharedDir ()
{
  std::ostringstream dirname;
  char* musicSharedDir = getenv ("MUSIC_SHARED_DIR");
  if (musicSharedDir)
    dirname << musicSharedDir;
  else
    {
      char* home = getenv ("HOME");
      dirname << home;
    }
  return dirname.str ();
}
#endif


std::string
getFirstArg (int argc, char** argv)
{
  for (int i = 1; i < argc; ++i)
    if (*argv[i] != '-') // skip options
      return argv[i];
  return "";
}


/*
 * Retrieve first argument (the name of the music configuration file)
 * given to the music utility.
 *
 * Be robust against rank == -1 (which occurs when getRank fails).
 */

std::istream*
getConfig (int rank, int argc, char** argv)
{
#ifdef MPICH
  std::ostringstream fname;
  fname << getSharedDir () << "/.musicconf";
  std::string confname;
  if (rank == 0)
    {
      std::ofstream f (fname.str ().c_str ());
      confname = getFirstArg (argc, argv);
      f << confname;
    }
  else if (rank > 0)
    {
      std::ifstream f (fname.str ().c_str ());
      f >> confname;
    }
  else // rank == -1
    confname = getFirstArg (argc, argv);
  return new std::ifstream (confname.c_str ());
#else
  return new std::ifstream (getFirstArg (argc, argv).c_str ());
#endif
}
