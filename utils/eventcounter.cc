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
      std::cerr << "Usage: eventcounter [OPTION...] N_UNITS PREFIX [SUFFIX]" << std::endl
		<< "`eventcounter' receives spikes through a MUSIC input port, counts" << std::endl
		<< "all spikes for each index and writes the frequencies to a set of files" << std::endl
		<< "with names PREFIX RANK SUFFIX" << std::endl << std:: endl
		<< "  -t, --timestep TIMESTEP time between tick() calls (default " << DEFAULT_TIMESTEP << " s)" << std::endl
		<< "  -b, --maxbuffered TICKS maximal amount of data buffered" << std::endl
		<< "  -m, --imaptype TYPE     linear (default) or roundrobin" << std::endl
		<< "  -h, --help              print this help message" << std::endl << std::endl
		<< "Report bugs to <music-bugs@incf.org>." << std::endl;
    }
  exit (1);
}

std::vector<int> counters;

class MyEventHandlerLocal: public MUSIC::EventHandlerLocalIndex {
public:
  void operator () (double t, MUSIC::LocalIndex id)
  {
    ++counters[id];
  }
};

int nUnits;
double timestep = DEFAULT_TIMESTEP;
int    maxbuffered = 0;
string imaptype = "linear";
string prefix;
string suffix = ".dat";

void
getargs (int rank, int argc, char* argv[])
{
  opterr = 0; // handle errors ourselves
  while (1)
    {
      static struct option longOptions[] =
	{
	  {"timestep",  required_argument, 0, 't'},
	  {"maxbuffered", required_argument, 0, 'b'},
	  {"imaptype",  required_argument, 0, 'm'},
	  {"help",      no_argument,       0, 'h'},
	  {0, 0, 0, 0}
	};
      /* `getopt_long' stores the option index here. */
      int option_index = 0;

      // the + below tells getopt_long not to reorder argv
      int c = getopt_long (argc, argv, "+t:b:m:h", longOptions, &option_index);

      /* detect the end of the options */
      if (c == -1)
	break;

      switch (c)
	{
	case 't':
	  timestep = atof (optarg); // NOTE: could do error checking
	  continue;
	case 'b':
	  maxbuffered = atoi (optarg);
	  continue;
	case 'm':
	  imaptype = optarg;
	  if (imaptype != "linear" && imaptype != "roundrobin")
	    {
	      usage (rank);
	      abort ();
	    }
	  continue;
	case '?':
	  break; // ignore unknown options
	case 'h':
	  usage (rank);

	default:
	  abort ();
	}
    }

  if (argc < optind + 2 || argc > optind + 3)
    usage (rank);

  nUnits = atoi (argv[optind]);
  prefix = argv[optind + 1];
  if (argc == optind + 3)
    suffix = argv[optind + 2];
}

int
main (int argc, char *argv[])
{
  MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);
  
  MPI_Comm comm = setup->communicator ();
  int nProcesses;
  MPI_Comm_size (comm, &nProcesses);
  int rank;
  MPI_Comm_rank (comm, &rank);
  
  getargs (rank, argc, argv);

  MUSIC::EventInputPort* in = setup->publishEventInput ("in");
  if (!in->isConnected ())
    {
      if (rank == 0)
	std::cerr << "eventcounter port is not connected" << std::endl;
      MPI_Abort (comm, 1);
    }

  std::ostringstream spikefile;
  spikefile << prefix << rank << suffix;
  std::ofstream out (spikefile.str ().c_str ());
  if (!out)
    {
      std::cerr << "eventcounter: could not open "
		<< spikefile.str () << " for writing" << std::endl;
      abort ();      
    }

  MyEventHandlerLocal evhandlerLocal;
  
  if (imaptype == "linear")
    {
      int nUnitsPerProcess = nUnits / nProcesses;
      int nLocalUnits = nUnitsPerProcess;
      int rest = nUnits % nProcesses;
      int firstId = nUnitsPerProcess * rank;
      if (rank < rest)
	{
	  firstId += rank;
	  nLocalUnits += 1;
	}
      else
	firstId += rest;
      MUSIC::LinearIndex indices (firstId, nLocalUnits);

      if (maxbuffered > 0)
	in->map (&indices, &evhandlerLocal, 0.0, maxbuffered);
      else
	in->map (&indices, &evhandlerLocal, 0.0);
    }
  else
    {
      std::vector<MUSIC::GlobalIndex> v;
      for (int i = rank; i < nUnits; i += nProcesses)
	v.push_back (i);
      MUSIC::PermutationIndex indices (&v.front (), v.size ());

      if (maxbuffered > 0)
	in->map (&indices, &evhandlerLocal, 0.0, maxbuffered);
      else
	in->map (&indices, &evhandlerLocal, 0.0);
    }

  counters.resize (nUnits);
  for (int i = 0; i < nUnits; ++i)
    counters[i] = 0;

  double stoptime;
  setup->config ("stoptime", &stoptime);

  MUSIC::Runtime* runtime = new MUSIC::Runtime (setup, timestep);

  double time = runtime->time ();
  while (time < stoptime)
    {
      // Retrieve data from other program
      runtime->tick ();
      
      time = runtime->time ();
    }
  for (std::vector<int>::iterator i = counters.begin ();
       i != counters.end ();
       ++i)
    out << static_cast<double> (*i) / stoptime << std::endl;
  out.close ();

  runtime->finalize ();

  delete runtime;

  return 0;
}
