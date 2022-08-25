/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2008, 2009, 2019, 2022 INCF
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
#include <cmath>

extern "C" {
#include <unistd.h>
#include <getopt.h>
}

#include <music.hh>
#include <music/error.hh>

const double DEFAULT_TIMESTEP = 1e-2;
const double DEFAULT_FREQUENCY = 10.0; // Hz

void
usage (int rank)
{
  if (rank == 0)
    {
      std::cerr << "Usage: multiport [OPTION...]" << std::endl
		<< "`eventgenerator' generates spikes from a Poisson distribution." << std::endl << std:: endl
		<< "  -t, --timestep  TIMESTEP time between tick() calls (default " << DEFAULT_TIMESTEP << " s)" << std::endl
		<< "  -b, --maxbuffered TICKS  maximal amount of data buffered" << std::endl
		<< "  -f, --frequency FREQ     average frequency (default " << DEFAULT_FREQUENCY << " Hz)" << std::endl
		<< "  -m, --imaptype  TYPE     linear (default) or roundrobin" << std::endl
		<< "  -i, --indextype TYPE     global (default) or local" << std::endl
		<< "      --out NAME:WIDTH     name and width of an output port" << std::endl
		<< "      --in  NAME:WIDTH     name and width of input port" << std::endl
		<< "  -h, --help               print this help message" << std::endl << std::endl
		<< "Report bugs to <music-bugs@incf.org>." << std::endl;
    }
  exit (1);
}

double timestep = DEFAULT_TIMESTEP;
int    maxbuffered = 0;
double freq = DEFAULT_FREQUENCY;
string imaptype = "linear";
string indextype = "global";

double
negexp (double m)
{
  return - m * log (drand48 ());
}

class Port {
protected:
  MUSIC::Setup* setup_;
  std::string name_;
  int width_;

public:
  Port (MUSIC::Setup* setup, std::string name, int width)
    : setup_ (setup), name_ (name), width_ (width) { }
};

class OutputPort : public Port {
  MUSIC::EventOutputPort* port_;
  MUSIC::Index::Type type;
  std::vector<MUSIC::GlobalIndex> ids;
  std::vector<double> nextSpike;
  double m;

public:
  OutputPort (MUSIC::Setup* setup, std::string name, int width)
    : Port (setup, name, width) { }

  void publish ()
  {
    port_ = setup_->publishEventOutput (name_);
    if (!port_->isConnected ())
      {
	if (MUSIC::mpi_get_rank (setup_->communicator ()) == 0)
	  std::cerr << "multiport port is not connected" << std::endl;
	MPI_Abort (setup_->communicator (), 1);
      }
  }

  void map (int maxbuffered, std::string imaptype, std::string indextype)
  {
    MPI_Comm comm = setup_->communicator ();
    int rank = MUSIC::mpi_get_rank (comm);
    int nProcesses = MUSIC::mpi_get_comm_size (comm);

    if (indextype == "global")
      type = MUSIC::Index::GLOBAL;
    else
      type = MUSIC::Index::LOCAL;
    
    if (imaptype == "linear")
      {
	int nUnitsPerProcess = width_ / nProcesses;
	int nLocalUnits = nUnitsPerProcess;
	int rest = width_ % nProcesses;
	int firstId = nUnitsPerProcess * rank;
	if (rank < rest)
	  {
	    firstId += rank;
	    nLocalUnits += 1;
	  }
	else
	  firstId += rest;
	for (int i = 0; i < nLocalUnits; ++i)
	  ids.push_back (firstId + i);
	MUSIC::LinearIndex indices (firstId, nLocalUnits);
      
	if (maxbuffered > 0)
	  port_->map (&indices, type, maxbuffered);
	else
	  port_->map (&indices, type);
      }
    else
      {
	for (int i = rank; i < width_; i += nProcesses)
	  ids.push_back (i);
	MUSIC::PermutationIndex indices (&ids.front (), ids.size ());
	if (maxbuffered > 0)
	  port_->map (&indices, type, maxbuffered);
	else
	  port_->map (&indices, type);
      }
  }

  void init (double freq)
  {
    m = 1.0 / freq;
    for (unsigned int i = 0; i < ids.size (); ++i)
      nextSpike.push_back (negexp (m));
  }

  void tick (double nextTime)
  {
    if (type == MUSIC::Index::GLOBAL)
      {
	for (unsigned int i = 0; i < ids.size (); ++i)
	  while (nextSpike[i] < nextTime)
	    {
	      port_->insertEvent (nextSpike[i],
				  MUSIC::GlobalIndex (ids[i]));
	      nextSpike[i] += negexp (m);
	    }
      }
    else
      {
	for (unsigned int i = 0; i < ids.size (); ++i)
	  while (nextSpike[i] < nextTime)
	    {
	      port_->insertEvent (nextSpike[i],
				  MUSIC::LocalIndex (i));
	      nextSpike[i] += negexp (m);
	    }
      }
  }
};

class MyEventHandlerGlobal : public MUSIC::EventHandlerGlobalIndex {
public:
  void operator () (double t, MUSIC::GlobalIndex id)
  {
    //eventBuffer.push_back (MUSIC::Event (t, id));
  }
};

class MyEventHandlerLocal: public MUSIC::EventHandlerLocalIndex {
public:
  void operator () (double t, MUSIC::LocalIndex id)
  {
    //eventBuffer.push_back (MUSIC::Event (t, id));
  }
};

class InputPort : public Port {
  MUSIC::EventInputPort* port_;
  MyEventHandlerGlobal evhandlerGlobal;
  MyEventHandlerLocal evhandlerLocal;

public:
  InputPort (MUSIC::Setup* setup, std::string name, int width)
    : Port (setup, name, width) { }

  void publish ()
  {
    port_ = setup_->publishEventInput (name_);
    if (!port_->isConnected ())
      {
	if (MUSIC::mpi_get_rank (setup_->communicator ()) == 0)
	  std::cerr << "multiport port is not connected" << std::endl;
	MPI_Abort (setup_->communicator (), 1);
      }
  }

  void map (std::string imaptype, std::string indextype)
  {
    MPI_Comm comm = setup_->communicator ();
    int rank = MUSIC::mpi_get_rank (comm);
    int nProcesses = MUSIC::mpi_get_comm_size (comm);

    if (imaptype == "linear")
      {
	int nUnitsPerProcess = width_ / nProcesses;
	int nLocalUnits = nUnitsPerProcess;
	int rest = width_ % nProcesses;
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
	  port_->map (&indices, &evhandlerGlobal, 0.1);
	else
	  port_->map (&indices, &evhandlerLocal, 0.0);
      }
    else
      {
	std::vector<MUSIC::GlobalIndex> v;
	for (int i = rank; i < width_; i += nProcesses)
	  v.push_back (i);
	MUSIC::PermutationIndex indices (&v.front (), v.size ());

	if (indextype == "global")
	  port_->map (&indices, &evhandlerGlobal, 0.0);
	else
	  port_->map (&indices, &evhandlerLocal, 0.0);
      }
  }
};

std::vector<OutputPort*> outputPort;

void
makeOutput (MUSIC::Setup* setup, std::string name, int width)
{
  outputPort.push_back (new OutputPort (setup, name, width));
}

std::vector<InputPort*> inputPort;

void
makeInput (MUSIC::Setup* setup, std::string name, int width)
{
  inputPort.push_back (new InputPort (setup, name, width));
}

void
parsePort (MUSIC::Setup* setup,
	   std::string s,
	   void (*makePort) (MUSIC::Setup* setup, std::string name, int width))
{
  std::istringstream is (s);
  char name[80];
  if (!is.getline (name, 80, ':'))
    MUSIC::error ("couldn't parse port name");
  int width;
  if (!(is >> width))
    MUSIC::error ("couldn't parse port width");
  makePort (setup, name, width);
}

void
getargs (MUSIC::Setup* setup, int argc, char* argv[])
{
  int rank = MUSIC::mpi_get_rank (setup->communicator ());

  enum { OUT, IN };
  opterr = 0; // handle errors ourselves
  while (1)
    {
      static struct option longOptions[] =
	{
	  {"timestep",  required_argument, 0, 't'},
	  {"maxbuffered", required_argument, 0, 'b'},
	  {"frequency", required_argument, 0, 'f'},
	  {"imaptype",  required_argument, 0, 'm'},
	  {"indextype", required_argument, 0, 'i'},
	  {"help",      no_argument,       0, 'h'},
	  {"out",	required_argument, 0, OUT},
	  {"in",	required_argument, 0, IN},
	  {0, 0, 0, 0}
	};
      /* `getopt_long' stores the option index here. */
      int option_index = 0;

      // the + below tells getopt_long not to reorder argv
      int c = getopt_long (argc, argv, "+t:b:f:m:i:h", longOptions, &option_index);

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
	case 'f':
	  freq = atof (optarg); // NOTE: error checking here as well
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
	case OUT:
	  parsePort (setup, optarg, makeOutput);
	  continue;
	case IN:
	  parsePort (setup, optarg, makeInput);
	  continue;
	case '?':
	  break; // ignore unknown options
	case 'h':
	  usage (rank);

	default:
	  abort ();
	}
    }

  if (argc != optind)
    usage (rank);
}

#include <cassert>

void
errhandler (MPI_Comm* comm, int* error, ...)
{
  assert (0);
}

int
main (int argc, char *argv[])
{
  MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);
  MPI_Errhandler errh;
#if MUSIC_HAVE_MPI_COMM_CREATE_ERRHANDLER
  MPI_Comm_create_errhandler (errhandler, &errh);
#else
  MPI_Errhandler_create (errhandler, &errh);
#endif
  MPI_Comm_set_errhandler (MPI_COMM_WORLD, errh);
  
  MPI_Comm comm = setup->communicator ();
  int rank = MUSIC::mpi_get_rank (comm);
  
  getargs (setup, argc, argv);

  for (std::vector<InputPort*>::iterator i = inputPort.begin ();
       i != inputPort.end ();
       ++i)
    (*i)->publish ();

  for (std::vector<OutputPort*>::iterator i = outputPort.begin ();
       i != outputPort.end ();
       ++i)
    (*i)->publish ();

  for (std::vector<InputPort*>::iterator i = inputPort.begin ();
       i != inputPort.end ();
       ++i)
    (*i)->map (imaptype, indextype);

  for (std::vector<OutputPort*>::iterator i = outputPort.begin ();
       i != outputPort.end ();
       ++i)
    (*i)->map (maxbuffered, imaptype, indextype);

  double stoptime;
  setup->config ("stoptime", &stoptime);

  MUSIC::Runtime* runtime = new MUSIC::Runtime (setup, timestep);

  srand48 (rank);		// Use different seeds

  for (std::vector<OutputPort*>::iterator i = outputPort.begin ();
       i != outputPort.end ();
       ++i)
    (*i)->init (freq);

  double time = runtime->time ();
  while (time < stoptime)
    {
      double nextTime = time + timestep;

      for (std::vector<OutputPort*>::iterator i = outputPort.begin ();
	   i != outputPort.end ();
	   ++i)
	(*i)->tick (nextTime);
      
      // Make data available for other programs
      runtime->tick ();

      time = runtime->time ();
    }

  runtime->finalize ();

  delete runtime;

  return 0;
}
