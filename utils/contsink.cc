#include <mpi.h>
#include <music.hh>
#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>

extern "C" {
#include <unistd.h>
#include <getopt.h>
}

#define DEFAULT_TIMESTEP 1e-2

double *data;


void
usage (int rank)
{
  if (rank == 0)
    {
      std::cerr << "Usage: contsink [OPTION...]" << std::endl
		<< "`contsink' receives continuous data" << std::endl
		<< "through a MUSIC input port." << std::endl << std:: endl
		<< "  -t, --timestep TIMESTEP time between tick() calls (default " << DEFAULT_TIMESTEP << " s)" << std::endl
		<< "  -d, --delay SECS        delay until data should arrive" << std::endl
		<< "  -i, --interpolate       no interpolation" << std::endl
		<< "  -h, --help              print this help message" << std::endl << std::endl
		<< "Report bugs to <music-bugs@incf.org>." << std::endl;
    }
  exit (1);
}

double timestep = DEFAULT_TIMESTEP;
double delay = 0.0;
bool   interpolate = true;

void
getargs (int rank, int argc, char* argv[])
{
  opterr = 0; // handle errors ourselves
  while (1)
    {
      static struct option longOptions[] =
	{
	  {"timestep",    required_argument, 0, 't'},
	  {"delay",       required_argument, 0, 'd'},
	  {"interpolate", no_argument,       0, 'i'},
	  {"help",        no_argument,       0, 'h'},
	  {0, 0, 0, 0}
	};
      /* `getopt_long' stores the option index here. */
      int option_index = 0;

      // the + below tells getopt_long not to reorder argv
      int c = getopt_long (argc, argv, "+t:d:ih",
			   longOptions, &option_index);

      /* detect the end of the options */
      if (c == -1)
	break;

      switch (c)
	{
	case 't':
	  timestep = atof (optarg);
	  continue;
	case 'd':
	  delay = atof (optarg);
	  continue;
	case 'i':
	  interpolate = false;
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
main (int argc, char* argv[])
{
  MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

  MUSIC::ContInputPort* contdata = setup->publishContInput ("contdata");

  MPI_Comm comm = setup->communicator ();
  int nProcesses;
  MPI_Comm_size (comm, &nProcesses);
  int rank;
  MPI_Comm_rank (comm, &rank);

  getargs (rank, argc, argv);

  int totalWidth = contdata->width ();
  int localWidth = (totalWidth-1) / nProcesses + 1;
  int myWidth = localWidth;
  if (rank == nProcesses - 1)	// Last processor
    myWidth = totalWidth - (nProcesses-1) * localWidth;

  data = new double[myWidth];

  // Declare where in memory to put data
  MUSIC::ArrayData dmap (data,
			 MPI_DOUBLE,
			 rank * localWidth,
			 myWidth);
  contdata->map (&dmap, delay, interpolate);

  double stoptime;
  setup->config ("stoptime", &stoptime);

  MUSIC::Runtime* runtime = new MUSIC::Runtime (setup, timestep);


  for (; runtime->time () < stoptime; runtime->tick ())
    {
      for (int i = 0; i < myWidth; ++i)
	std::cout << data[i] << " ";
      std::cout << "on " << rank << " @" << runtime->time () << std::endl;
    }

  runtime->finalize ();
  
  delete runtime;

  return 0;
}
