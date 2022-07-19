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
#include <fstream>
#include <string>
#include <cstdlib>
#include <vector>

extern "C" {
#include <unistd.h>
#include <getopt.h>
}

#include <music.hh>

const double DEFAULT_TIMESTEP = 1e-2;

void
usage (int rank)
{
  if (rank == 0)
    {
      std::cerr << "Usage: contdelay [OPTION...]" << std::endl
		<< "`contdelay' receives data a MUSIC input port" << std::endl
		<< "and sends them out after a delay" << std::endl << std:: endl
		<< "  -t, --timestep TIMESTEP time between tick() calls (default " << DEFAULT_TIMESTEP << " s)" << std::endl
		<< "  -d, --delay SECS        amount of delay" << std::endl
		<< "  -b, --maxbuffer TICKS   maximal buffer" << std::endl
		<< "  -h, --help              print this help message" << std::endl << std::endl
		<< "Report bugs to <music-bugs@incf.org>." << std::endl;
    }
  exit (1);
}

std::vector<MUSIC::Event> eventBuffer;
std::vector<MUSIC::Event> overflowBuffer;

double timestep = DEFAULT_TIMESTEP;
double delay = 0.0;
int label = -1;
int maxbuffered = 0;


void
getargs (int rank, int argc, char* argv[])
{
  opterr = 0; // handle errors ourselves
  while (1)
    {
      static struct option longOptions[] =
	{
	  {"timestep",  required_argument, 0, 't'},
	  {"delay",     required_argument, 0, 'd'},
	  {"label",     required_argument, 0, 'L'},
	  {"maxbuffer", required_argument, 0, 'b'},
	  {"help",      no_argument,       0, 'h'},
	  {0, 0, 0, 0}
	};
      /* `getopt_long' stores the option index here. */
      int option_index = 0;

      // the + below tells getopt_long not to reorder argv
      int c = getopt_long (argc, argv, "+t:d:L:b:h", longOptions, &option_index);

      /* detect the end of the options */
      if (c == -1)
	break;

      switch (c)
	{
	case 't':
	  timestep = atof (optarg);
	  continue;
	case 'd':
	  delay = atof(optarg);
	  continue;
	case 'L':
	  label = atoi(optarg);
	  continue;
	case 'b':
	  maxbuffered = atoi(optarg);
	  continue;
	case '?':
	  break; // ignore unknown options
	case 'h':
	  usage (rank);

	default:
	  abort ();
	}
    }

  if (argc < optind || argc > optind)
    usage (rank);
}

double inData = 0.0;
double outData = 0.0;
double auxData = 0.0;


int
main (int argc, char *argv[])
{
  MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

  MPI_Comm comm = setup->communicator ();
  int rank;
  MPI_Comm_rank (comm, &rank);
  
  getargs (rank, argc, argv);

  MUSIC::ContInputPort* in = setup->publishContInput ("in");
  if (!in->isConnected ())
    {
      if (rank == 0)
	std::cerr << "contdelay input port is not connected" << std::endl;
      MPI_Abort (comm, 1);
    }

  MUSIC::ArrayData inMap (&inData,
			  MPI_DOUBLE,
			  0,
			  1);
  in->map (&inMap, delay, maxbuffered);


  MUSIC::ContOutputPort* out = setup->publishContOutput ("out");
  if (!out->isConnected ())
    {
      if (rank == 0)
	std::cerr << "contdelay output port is not connected" << std::endl;
      MPI_Abort (comm, 1);
    }

  MUSIC::ArrayData outMap (&outData,
			   MPI_DOUBLE,
			   0,
			   1);
  out->map (&outMap, maxbuffered);


  // Optional extra input port
  MUSIC::ContInputPort* aux = setup->publishContInput ("aux");

  if (aux->isConnected ())
    {
      MUSIC::ArrayData auxMap (&auxData,
			      MPI_DOUBLE,
			      0,
			      1);
      aux->map (&auxMap, delay, maxbuffered);
    }

  double stoptime;
  setup->config ("stoptime", &stoptime);

  MUSIC::Runtime* runtime = new MUSIC::Runtime (setup, timestep);

  for (; runtime->time () < stoptime; runtime->tick ())
    {
      outData = inData + auxData;
    }

  runtime->finalize ();

  delete runtime;

  return 0;
}
