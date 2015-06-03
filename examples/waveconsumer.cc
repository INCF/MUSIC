#include <mpi.h>
#include <music.hh>
#include <fstream>
#include <sstream>

#define TIMESTEP 0.0005

MPI::Intracomm comm;
double* data;

int
main (int args, char* argv[])
{
  MUSIC::Setup* setup = new MUSIC::Setup (args, argv);

  MUSIC::ContInputPort* wavedata =
    setup->publishContInput ("wavedata");

  comm = setup->communicator ();
  int nProcesses = comm.Get_size (); // how many processes are there?
  int rank = comm.Get_rank ();       // which process am I?
  int width = 0;
  if (wavedata->hasWidth ())
    width = wavedata->width ();
  else
    comm.Abort (1);

  // For clarity, assume that width is a multiple of n_processes
  int nLocalVars = width / nProcesses;
  data = new double[nLocalVars];
  std::ostringstream filename;
  filename << argv[1] << rank << ".out";
  std::ofstream file (filename.str ().data ());
    
  // Declare where in memory to put data
  MUSIC::ArrayData dmap (data,
			 MPI::DOUBLE,
			 rank * nLocalVars,
			 nLocalVars);
  wavedata->map (&dmap);

  double stoptime;
  setup->config ("stoptime", &stoptime);
  MUSIC::Runtime* runtime = new MUSIC::Runtime (setup, TIMESTEP);
  for (int j =0; runtime->time () < stoptime; j++)
    {
	  runtime->tick ();
      // Dump to file
      for (int i = 0; i < nLocalVars; ++i)
    	  file << data[i] << ' ';
      file << std::endl;

    }
  runtime->finalize ();
  
  delete runtime;

  return 0;
}
