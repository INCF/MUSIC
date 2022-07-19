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
      std::cerr << "Usage: eventselect [OPTION...] N_UNITS UNITS" << std::endl
		<< "`eventselect' receives events from an input port of width N_UNITS" << std::endl
		<< "and sends events for the subset of id:s specified in the file UNITS" << std::endl << std:: endl
		<< "  -t, --timestep TIMESTEP time between tick() calls (default " << DEFAULT_TIMESTEP << " s)" << std::endl
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

int nUnits;
double timestep = DEFAULT_TIMESTEP;
string units;

void
getargs (int rank, int argc, char* argv[])
{
  opterr = 0; // handle errors ourselves
  while (1)
    {
      static struct option longOptions[] =
	{
	  {"timestep",  required_argument, 0, 't'},
	  {"help",      no_argument,       0, 'h'},
	  {0, 0, 0, 0}
	};
      /* `getopt_long' stores the option index here. */
      int option_index = 0;

      // the + below tells getopt_long not to reorder argv
      int c = getopt_long (argc, argv, "+t:m:i:h", longOptions, &option_index);

      /* detect the end of the options */
      if (c == -1)
	break;

      switch (c)
	{
	case 't':
	  timestep = atof (optarg); // NOTE: some error checking would be good!
	  continue;
	case '?':
	  break; // ignore unknown options
	case 'h':
	  usage (rank);

	default:
	  abort ();
	}
    }

  if (argc != optind + 2)
    usage (rank);

  nUnits = atoi (argv[optind]);
  units = argv[optind + 1];
}

void
mapOutput (MUSIC::EventOutputPort* out,
	   int nProcesses,
	   int rank,
	   std::string fname)
{
  std::ifstream unitfile (fname.c_str ());
  if (!unitfile)
    {
      std::cerr << "eventselect: could not open "
		<< fname << " for writing" << std::endl;
      abort ();      
    }

  // First count the integers
  int nUnits = 0;
  int id;
  while (unitfile >> id)
    ++nUnits;
  unitfile.close ();
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
  // Now read in our selection
  nUnits = 0;
  unitfile.open (fname.c_str ());
  while (nUnits < firstId)
    {
      unitfile >> id;
      ++nUnits;
    }
  nUnits = 0;
  std::vector<MUSIC::GlobalIndex> units;
  while (nUnits < nLocalUnits)
    {
      unitfile >> id;
      units.push_back (id);
      ++nUnits;
    }
  unitfile.close ();
  MUSIC::PermutationIndex indices (&units.front (), units.size ());

  out->map (&indices, MUSIC::Index::GLOBAL);
}

void
mapInput (MUSIC::EventInputPort* in,
	  int nProcesses,
	  int rank,
	  MyEventHandlerGlobal& evhandler)
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

  in->map (&indices, &evhandler, 0.0);
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
	std::cerr << "eventselect port in is not connected" << std::endl;
      MPI_Abort (comm, 1);
    }

  MUSIC::EventOutputPort* out = setup->publishEventOutput ("out");
  if (!out->isConnected ())
    {
      if (rank == 0)
	std::cerr << "eventselect port out is not connected" << std::endl;
      MPI_Abort (comm, 1);
    }

  mapOutput (out, nProcesses, rank, units);

  MyEventHandlerGlobal evhandlerGlobal;

  mapInput (in, nProcesses, rank, evhandlerGlobal);
  
  double stoptime;
  setup->config ("stoptime", &stoptime);

  MUSIC::Runtime* runtime = new MUSIC::Runtime (setup, timestep);

  double time = runtime->time ();
  while (time < stoptime)
    {
      eventBuffer.clear ();
      // Retrieve data
      runtime->tick ();
      
      sort (eventBuffer.begin (), eventBuffer.end ());
      for (std::vector<MUSIC::Event>::iterator i = eventBuffer.begin ();
	   i != eventBuffer.end ();
	   ++i)
	// Send data (NOTE: assumes that non-mapped id:s will be discarded)
	out->insertEvent (i->t, MUSIC::GlobalIndex (i->id));

      time = runtime->time ();
    }

  runtime->finalize ();

  delete runtime;

  return 0;
}
