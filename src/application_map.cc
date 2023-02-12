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


#include "music/application_map.hh"
#if MUSIC_USE_MPI
#include "music/mpi_utils.hh"
#endif
#include "music/ioutils.hh"
#include <iostream>
#include <fstream>
namespace MUSIC {

  ApplicationMap::ApplicationMap ()
  {
  }

  int
  ApplicationMap::nProcesses ()
  {
    int n = 0;
    for (ApplicationMap::iterator i = begin (); i != end (); ++i)
      n += i->nProc ();
    return n;
  }
  
  
  ApplicationInfo*
  ApplicationMap::lookup (std::string appName)
  {
    for (iterator i = begin (); i != end (); ++i)
      {
	if (i->name () == appName)
	  return &*i;
      }
    return 0;
  }


  void
  ApplicationMap::add (std::string name, int n, int c)
  {
    push_back (ApplicationInfo (name, n, c));
  }

  
  void
  ApplicationMap::write (std::ostringstream& out)
  {
	  out << size ();
	  for (iterator i = begin (); i != end (); ++i)
	  {
		  out << ':';
		  IOUtils::write (out, i->name ());
		  out << ':' << i->nProc ();
	  }
  }


  void
  ApplicationMap::read (std::istringstream& in)
  {
    int nApp;
    in >> nApp;

#ifdef MUSIC_DEBUG
    /*  for debugging */
    std::ofstream outfile ("leaders");
#endif
    for (int i = 0; i < nApp; ++i)
      {
        in.ignore ();
        std::string name = IOUtils::read (in);
        in.ignore ();
        int np;
        in >> np;
#ifdef MUSIC_DEBUG
        /*  for debugging */
        outfile<< name << "\t" << aleader << std::endl;
#endif
        add (name, np, i);
      }
#ifdef MUSIC_DEBUG
    /*  for debugging */
    outfile.close();
#endif
  }


  ApplicationInfo*
  ApplicationMap::lookup (int color)
  {
    for (iterator i = begin (); i != end (); ++i)
      {
        if (i->color () == color)
          return &*i;
      }
    return 0;

  }

#if MUSIC_USE_MPI
  std::map<int, int>
  ApplicationMap::assignLeaders (std::string my_app_label)
  {
    std::map<int, int> leaders;

    int size = mpi_get_comm_size (MPI_COMM_WORLD);
    int rank = mpi_get_rank (MPI_COMM_WORLD);
    int *colors = new int[size];
    colors[rank] = lookup (my_app_label)->color ();
    MPI_Allgather (MPI_IN_PLACE, 0, MPI_INT, colors, 1, MPI_INT, MPI_COMM_WORLD);

    int prev_color = -1;
    for (int i = 0; i < size; ++i)
      {
        if (colors[i] != prev_color)
          {
            ApplicationInfo* info = lookup (colors[i]);
            info->setLeader (i);
            leaders[info->color ()] = i;
            prev_color = colors[i];
          }
      }

#ifdef MUSIC_DEBUG
    /*  block for debugging */
    std::ofstream outfile ("ranks");
    for(int i = 0; i < nLeaders; ++i)
      {
        outfile<< i <<":";
        for(int j = 0; j < gsize; ++j)
        if(rbuf[j] == i) outfile<< " " << j;
        outfile<<std::endl;
      }
    outfile<<std::endl;
    outfile.close();
    /*end of block */
#endif // MUSIC_DEBUG
    delete[] colors;
    return leaders;
  }
#endif
}

