/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2008, 2009, 2018, 2022 INCF
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
      std::cerr << "Usage: eventsink [OPTION...] N_UNITS PREFIX [SUFFIX]" << std::endl
		<< "`eventsink' receives spikes through a MUSIC input port" << std::endl
		<< "and writes these to a set of files with names PREFIX RANK SUFFIX" << std::endl << std:: endl
		<< "  -t, --timestep TIMESTEP time between tick() calls (default " << DEFAULT_TIMESTEP << " s)" << std::endl
		<< "  -m, --imaptype TYPE     linear (default) or roundrobin" << std::endl
		<< "  -i, --indextype TYPE    global (default) or local" << std::endl
		<< "      --in PORTNAME       input port name (default: in)" << std::endl
		<< "  -h, --help              print this help message" << std::endl << std::endl
		<< "Report bugs to <music-bugs@incf.org>." << std::endl;
    }
  exit (1);
}

std::vector<MUSIC::Event> eventBuffer;

class MyEventHandlerGlobal : public MUSIC::EventHandlerGlobalIndex {
public:
  void operator () (double t, MUSIC::GlobalIndex id)
  {
    eventBuffer.push_back (MUSIC::Event (t, id));
  }
};

class MyEventHandlerLocal: public MUSIC::EventHandlerLocalIndex {
public:
  void operator () (double t, MUSIC::LocalIndex id)
  {
    eventBuffer.push_back (MUSIC::Event (t, id));
  }
};

string portName ("in");
int nUnits;
double timestep = DEFAULT_TIMESTEP;
string imaptype = "linear";
string indextype = "global";
string prefix;
string suffix = ".dat";
bool useBarrier = false;

void
getargs (int rank, int argc, char* argv[])
{
  enum { IN };
  opterr = 0; // handle errors ourselves
  while (1)
    {
      static struct option longOptions[] =
	{
	  {"timestep",  required_argument, 0, 't'},
	  {"imaptype",  required_argument, 0, 'm'},
	  {"indextype", required_argument, 0, 'i'},
	  {"help",      no_argument,       0, 'h'},
	  {"in",	required_argument, 0, IN},
	  {"adapter",	  no_argument, 0, 'z'},
	  {0, 0, 0, 0}
	};
      /* `getopt_long' stores the option index here. */
      int option_index = 0;

      // the + below tells getopt_long not to reorder argv
      int c = getopt_long (argc, argv, "+t:m:i:hz", longOptions, &option_index);

      /* detect the end of the options */
      if (c == -1)
	break;

      switch (c)
	{
	case 't':
	  timestep = atof (optarg); // NOTE: could do error checking
	  continue;
	case 'm':
	  imaptype = optarg;
	  if (imaptype != "linear" && imaptype != "roundrobin")
	    {
	      usage (rank);
	      abort ();
	    }
	  continue;
	case 'i':
	  indextype = optarg;
	  if (indextype != "global" && indextype != "local")
	    {
	      usage (rank);
	      abort ();
	    }
	  continue;
	case IN:
	  portName = optarg;
	  continue;
	case 'z':
	  useBarrier = true;
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

  MUSIC::EventInputPort* in = setup->publishEventInput (portName);
  if (!in->isConnected ())
    {
      if (rank == 0)
	std::cerr << "eventsink port is not connected" << std::endl;
      MPI_Abort (comm, 1);
    }

  std::ostringstream spikefile;
  spikefile << prefix << rank << suffix;
  std::ofstream out (spikefile.str ().c_str ());
  if (!out)
    {
      std::cerr << "eventsink: could not open "
		<< spikefile.str () << " for writing" << std::endl;
      abort ();      
    }

  MyEventHandlerGlobal evhandlerGlobal;
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

      if (indextype == "global")
	in->map (&indices, &evhandlerGlobal, 0.0);
      else
	in->map (&indices, &evhandlerLocal, 0.0);
    }
  else
    {
      std::vector<MUSIC::GlobalIndex> v;
      for (int i = rank; i < nUnits; i += nProcesses)
	v.push_back (i);
      MUSIC::PermutationIndex indices (&v.front (), v.size ());

      if (indextype == "global")
	in->map (&indices, &evhandlerGlobal, 0.0);
      else
	in->map (&indices, &evhandlerLocal, 0.0);
    }

  double stoptime;
  setup->config ("stoptime", &stoptime);

  if (useBarrier)
    MPI_Barrier (MPI_COMM_WORLD);

  MUSIC::Runtime* runtime = new MUSIC::Runtime (setup, timestep);

  double time = runtime->time ();
  while (time < stoptime)
    {
      eventBuffer.clear ();
      // Retrieve data from other program
      runtime->tick ();
      
      sort (eventBuffer.begin (), eventBuffer.end ());
      for (std::vector<MUSIC::Event>::iterator i = eventBuffer.begin ();
	   i != eventBuffer.end ();
	   ++i)
	out << i->t << '\t' << i->id << std::endl;

      time = runtime->time ();
    }
  out.close ();

  runtime->finalize ();

  delete runtime;

  return 0;
}
