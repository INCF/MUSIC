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

// Leave as first include---required by BG/L
#include <mpi.h>

#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>

extern "C" {
#include <unistd.h>
#include <getopt.h>
}

#include <music.hh>

const double DEFAULT_TIMESTEP = 1e-2;

double dataarray[1];

void
usage (int rank)
{
  if (rank == 0)
    {
      std::cerr << "Usage: clocksource [OPTION...]" << std::endl
		<< "`clocksource' sends out the current time" << std::endl
		<< "through a MUSIC output port." << std::endl << std:: endl
		<< "  -t, --timestep TIMESTEP time between tick() calls (default " << DEFAULT_TIMESTEP << " s)" << std::endl
		<< "  -b, --maxbuffered TICKS maximal amount of data buffered" << std::endl
		<< "  -h, --help              print this help message" << std::endl << std::endl
		<< "Report bugs to <music-bugs@incf.org>." << std::endl;
    }
  exit (1);
}

double timestep = DEFAULT_TIMESTEP;
int    maxbuffered = 0;


void
getargs (int rank, int argc, char* argv[])
{
  opterr = 0; // handle errors ourselves
  while (1)
    {
      static struct option longOptions[] =
	{
	  {"timestep",    required_argument, 0, 't'},
	  {"maxbuffered", required_argument, 0, 'b'},
	  {"help",        no_argument,       0, 'h'},
	  {0, 0, 0, 0}
	};
      /* `getopt_long' stores the option index here. */
      int option_index = 0;

      // the + below tells getopt_long not to reorder argv
      int c = getopt_long (argc, argv, "+t:b:h",
			   longOptions, &option_index);

      /* detect the end of the options */
      if (c == -1)
	break;

      switch (c)
	{
	case 't':
	  timestep = atof (optarg);
	  continue;
	case 'b':
	  maxbuffered = atoi (optarg);
	  continue;
	case '?':
	  break; // ignore unknown options
	case 'h':
	  usage (rank);

	default:
	  abort ();
	}
    }

  if (argc < optind + 0 || argc > optind + 0)
    usage (rank);
}

int
main (int argc, char *argv[])
{
  MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);
  
  MPI_Comm comm = setup->communicator ();
  int rank;
  MPI_Comm_rank (comm, &rank);
  
  getargs (rank, argc, argv);

  MUSIC::ContOutputPort* out = setup->publishContOutput ("clock");
  if (!out->isConnected ())
    {
      if (rank == 0)
	std::cerr << "clocksource port is not connected" << std::endl;
      MPI_Abort (comm, 1);
    }


  MUSIC::ArrayData dmap (dataarray,
			 MPI_DOUBLE,
			 0,
			 1);

  if (maxbuffered > 0)
    out->map (&dmap, maxbuffered);
  else
    out->map (&dmap);


  double stoptime;
  setup->config ("stoptime", &stoptime);

  dataarray[0] = 0.0;
  
  MUSIC::Runtime* runtime = new MUSIC::Runtime (setup, timestep);


  for (; runtime->time () < stoptime; runtime->tick ())
    {
      // Use current time (valid at next tick) at exported data
      dataarray[0] = runtime->time () + timestep;
    }

  runtime->finalize ();

  delete runtime;

  return 0;
}
