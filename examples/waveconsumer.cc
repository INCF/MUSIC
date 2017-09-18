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
  auto context = MUSIC::MusicContextFactory ().createContext (args, argv);
  MUSIC::Application app (context);

  auto wavePort= app.publish<MUSIC::ContInputPort> ("wavedata");

  comm = app.communicator ();
  int nProcesses = comm.Get_size (); // how many processes are there?
  int rank = comm.Get_rank ();       // which process am I?
  int width = 0;
  if (wavePort->hasWidth ())
    width = wavePort->width ();
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
  wavePort->map (&dmap);

  double stoptime;
  app.config ("stoptime", &stoptime);
  app.enterSimulationLoop (TIMESTEP);

  for (int j =0; app.time () < stoptime; j++)
    {
	  app.tick ();
      // Dump to file
      for (int i = 0; i < nLocalVars; ++i)
    	  file << data[i] << ' ';
      file << std::endl;

    }
  app.finalize ();

  return 0;
}
