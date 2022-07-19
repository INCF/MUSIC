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
      std::cerr << "Usage: eventdelay [OPTION...]" << std::endl
		<< "`eventdelay' receives spikes through a MUSIC input port" << std::endl
		<< "and relays them to an output port" << std::endl << std:: endl
		<< "  -t, --timestep TIMESTEP time between tick() calls (default " << DEFAULT_TIMESTEP << " s)" << std::endl
		<< "  -d, --delay SECS        amount of delay" << std::endl
		<< "  -b, --maxbuffer TICKS   maximal buffer" << std::endl
		<< "  -L, --label LABEL       log events using LABEL" << std::endl
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

#ifdef MUSIC_LOCAL
class MyEventHandler: public MUSIC::EventHandlerLocalIndex {
public:
  void operator () (double t, MUSIC::LocalIndex id)
  {
    eventBuffer.push_back (MUSIC::Event (t + delay, id));
    if (label != -1)
      std::cout << label << ":Got(" << id <<
	", " << t + delay << ")" << std::endl;
  }
};
#else
class MyEventHandler: public MUSIC::EventHandlerGlobalIndex {
public:
  void operator () (double t, MUSIC::GlobalIndex id)
  {
    eventBuffer.push_back (MUSIC::Event (t + delay, id));
    if (label != -1)
      std::cout << label << ":Got(" << id <<
	", " << t + delay << ")" << std::endl;
  }
};
#endif

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
	std::cerr << "eventdelay input port is not connected" << std::endl;
      MPI_Abort (comm, 1);
    }

  MUSIC::EventOutputPort* out = setup->publishEventOutput ("out");
  if (!out->isConnected ())
    {
      if (rank == 0)
	std::cerr << "eventdelay output port is not connected" << std::endl;
      MPI_Abort (comm, 1);
    }

  // Optional extra input port
  MUSIC::EventInputPort* aux = setup->publishEventInput ("aux");

  int width = in->width ();
    
  MyEventHandler evhandler;
  
  int localWidth = width / nProcesses;
  int myWidth = localWidth;

  if (rank == nProcesses-1)
    {
      myWidth = localWidth;	// FIXME
    }

  MUSIC::LinearIndex indices (rank*localWidth, myWidth);

  if (maxbuffered)
    in->map (&indices, &evhandler, delay, maxbuffered);
  else
    in->map (&indices, &evhandler, delay);

#ifdef MUSIC_LOCAL
  out->map (&indices, MUSIC::Index::LOCAL);
#else
  out->map (&indices, MUSIC::Index::GLOBAL);
#endif
  if (aux->isConnected ())
    {
      if (maxbuffered)
	aux->map (&indices, &evhandler, 0.0, maxbuffered);
      else
	aux->map (&indices, &evhandler, 0.0);
    }
  double stoptime;
  setup->config ("stoptime", &stoptime);

  MUSIC::Runtime* runtime = new MUSIC::Runtime (setup, timestep);

  for (; runtime->time () < stoptime; runtime->tick ())
    {
      sort (eventBuffer.begin (), eventBuffer.end ());
      for (std::vector<MUSIC::Event>::iterator i = eventBuffer.begin ();
	   i != eventBuffer.end ();
	   ++i)
	{
	  if (i->t < runtime->time () + timestep)
	    {
	      if (label != -1)
		std::cout << label << ":Sent(" << i->id << ", "
			  << i->t << " @" << runtime->time ()
			  << ")" << std::endl;
#ifdef MUSIC_LOCAL
	      out->insertEvent (i->t, MUSIC::LocalIndex (i->id));
#else
	      out->insertEvent (i->t, MUSIC::GlobalIndex (i->id));
#endif
	    }
	  else
	    overflowBuffer.push_back (*i);
	}
      eventBuffer.clear ();

      for (std::vector<MUSIC::Event>::iterator i = overflowBuffer.begin ();
	   i != overflowBuffer.end ();
	   ++i)
	eventBuffer.push_back (*i);
      overflowBuffer.clear ();
    }

  runtime->finalize ();

  delete runtime;

  return 0;
}
