#include <mpi.h>
#include <cstdlib>
#include <iostream>
#include <music.hh>

extern "C" {
#include <unistd.h>
#include <getopt.h>
}

const double DEFAULT_TIMESTEP = 0.01;

MPI_Comm comm;

void
usage (int rank)
{
  if (rank == 0)
    {
      std::cerr << "Usage: eventlogger [OPTION...]" << std::endl
		<< "`eventlogger' logs spikes from a MUSIC port." << std::endl << std::endl
		<< "  -t, --timestep TIMESTEP time between tick() calls (default " << DEFAULT_TIMESTEP << " s)" << std::endl
		<< "  -l, --acclatency LATENCY acceptable data latency (s)" << std::endl
		<< "  -b, --maxbuffered TICKS maximal amount of data buffered" << std::endl
		<< "  -m, --imaptype TYPE     linear (default) or roundrobin" << std::endl
		<< "  -i, --indextype TYPE    global (default) or local" << std::endl
		<< "  -a, --all               each receiver rank reports all spikes" << std::endl
		<< "      --in PORTNAME       input port name (default: in)" << std::endl
		<< "      --message-in PORTNAME message port name (default: message)" << std::endl
		<< "  -h, --help              print this help message" << std::endl << std::endl
		<< "Report bugs to <music-bugs@incf.org>." << std::endl;
    }
  exit (1);
}

double apptime;

class MyEventHandlerGlobal : public MUSIC::EventHandlerGlobalIndex {
  int rank;
public:
  MyEventHandlerGlobal (int rank_) : rank (rank_) { }
  void operator () (double t, MUSIC::GlobalIndex id)
  {
    // For now: just print out incoming events
    std::cout << "Rank " << rank
	      << ": Event (" << id << ", " << t
	      << ") detected at " << apptime << std::endl;
  }
};

class MyEventHandlerLocal: public MUSIC::EventHandlerLocalIndex {
  int rank;
public:
  MyEventHandlerLocal (int rank_) : rank (rank_) { }
  void operator () (double t, MUSIC::LocalIndex id)
  {
    // For now: just print out incoming events
    std::cout << "Rank " << rank
	      << ": Event (" << id << ", " << t
	      << ") detected at " << apptime << std::endl;
  }
};

class MyMessageHandler : public MUSIC::MessageHandler {
  int rank;
public:
  MyMessageHandler (int rank_) : rank (rank_) { }
  void operator () (double t, void* msg, size_t size)
  {
    // Print out incoming messages
    std::string message (static_cast<char*> (msg), size);
    std::cout << "Rank " << rank
	      << ": Message (" << t << ", " << message
	      << ") detected at " << apptime << std::endl;
  }
};

string portName ("in");
string messagePortName ("message");
double timestep = DEFAULT_TIMESTEP;
double latency = 0.0;
int    maxbuffered = 0;
std::string imaptype = "linear";
std::string indextype = "global";
bool all = false;
bool useBarrier = false;

int
main (int argc, char* argv[])
{
  MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);
  comm = setup->communicator ();
  int nProcesses;
  MPI_Comm_size (comm, &nProcesses); // how many processes are there?
  int rank;
  MPI_Comm_rank (comm, &rank); // which process am I?

  enum { IN, MESSAGE_IN };
  opterr = 0; // handle errors ourselves
  while (1)
    {
      static struct option longOptions[] =
	{
	  {"timestep",  required_argument, 0, 't'},
	  {"acclatency", required_argument, 0, 'l'},
	  {"maxbuffered", required_argument, 0, 'b'},
	  {"imaptype",  required_argument, 0, 'm'},
	  {"indextype", required_argument, 0, 'i'},
	  {"all",	no_argument,	   0, 'a'},
	  {"adapter",	no_argument,	   0, 'z'},
	  {"help",      no_argument,       0, 'h'},
	  {"in",	required_argument, 0, IN},
	  {"message-in", required_argument, 0, MESSAGE_IN},
	  {0, 0, 0, 0}
	};
       //`getopt_long' stores the option index here.
      int optionIndex = 0;

      // the + below tells getopt_long not to reorder argv
      int c = getopt_long (argc, argv, "+t:l:b:m:i:azh", longOptions, &optionIndex);

     //  detect the end of the options
      if (c == -1)
	break;

      switch (c)
	{
	case 't':
	  timestep = atof (optarg); // NOTE: could add error checking
	  continue;
	case 'l':
	  latency = atof (optarg);
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
	case 'i':
	  indextype = optarg;
	  if (indextype != "global" && indextype != "local")
	    {
	      usage (rank);
	      abort ();
	    }
	  continue;
	case 'a':
	  all = true;
	  continue;
	case 'z':
	  useBarrier = true;
	  continue;
	case IN:
	  portName = optarg;
	  continue;
	case MESSAGE_IN:
	  messagePortName = optarg;
	  continue;
	case '?':
	  break; // ignore unknown options
	case 'h':
	  usage (rank);

	default:
	  abort ();
	}
    }

  // Port publishing
  MUSIC::EventInputPort* evport = setup->publishEventInput (portName);
  MUSIC::MessageInputPort* msgport = setup->publishMessageInput (messagePortName);
  if (!evport->isConnected () && !msgport->isConnected ())
    {
      if (rank == 0)
	std::cerr << "eventlogger: no connected input ports" << std::endl;
      MPI_Abort (comm, 1);
    }

  // Port mapping

  MyEventHandlerGlobal evhandlerGlobal (rank);
  MyEventHandlerLocal evhandlerLocal (rank);

  if (evport->isConnected ())
    {
      // Split the width among the available processes
      int width = 0;
      if (evport->hasWidth ())
	width = evport->width ();
      else
	{
	  std::cerr << "port width not specified in Configuration file"
		    << std::endl;
	  MPI_Abort (comm, 1);
	}

      if (imaptype == "linear")
	{
	  int nLocal = width / nProcesses;
	  int rest = width % nProcesses;
	  int firstId = nLocal * rank;
	  if (rank < rest)
	    {
	      firstId += rank;
	      nLocal += 1;
	    }
	  else
	    firstId += rest;
	  MUSIC::LinearIndex indexmap (all ? 0 : firstId,
				       all ? width : nLocal);
      
	  if (indextype == "global")
	    if (maxbuffered > 0)
	      evport->map (&indexmap, &evhandlerGlobal, latency, maxbuffered);
	    else
	      evport->map (&indexmap, &evhandlerGlobal, latency);
	  else
	    if (maxbuffered > 0)
	      evport->map (&indexmap, &evhandlerLocal, latency, maxbuffered);
	    else
	      evport->map (&indexmap, &evhandlerLocal, latency);
	}
      else
	{
	  std::vector<MUSIC::GlobalIndex> v;
	  for (int i = all ? 0 : rank; i < width; i += all ? 1 : nProcesses)
	    v.push_back (i);
	  MUSIC::PermutationIndex indexmap (&v.front (), v.size ());
      
	  if (indextype == "global")
	    if (maxbuffered > 0)
	      evport->map (&indexmap, &evhandlerGlobal, latency, maxbuffered);
	    else
	      evport->map (&indexmap, &evhandlerGlobal, latency);
	  else
	    if (maxbuffered > 0)
	      evport->map (&indexmap, &evhandlerLocal, latency, maxbuffered);
	    else
	      evport->map (&indexmap, &evhandlerLocal, latency);
	}
    }

  // Messages
  MyMessageHandler msgHandler (rank);
  if (msgport->isConnected ())
    {
      if (maxbuffered > 0)
	msgport->map (&msgHandler, latency, maxbuffered);
      else
	msgport->map (&msgHandler, latency);
    }

  double stoptime;
  setup->config ("stoptime", &stoptime);

  if (useBarrier)
    MPI_Barrier (MPI_COMM_WORLD);

  // Run
  MUSIC::Runtime* runtime = new MUSIC::Runtime (setup, timestep);

  apptime = runtime->time ();
  while (apptime < stoptime)
    {
      // Retrieve data from other program
      runtime->tick ();

      apptime = runtime->time ();
    }
  runtime->finalize ();

  delete runtime;

  return 0;
}
