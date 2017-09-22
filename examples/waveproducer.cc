#include <mpi.h>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <music.hh>

#define TIMESTEP 0.001

MPI::Intracomm comm;
double* data;

int
main (int argc, char* argv[])
{
  auto context = MUSIC::MusicContextFactory ().createContext (argc, argv);
  MUSIC::Application app (std::move(context));

  auto wave_port = app.publish<MUSIC::ContOutputPort> ("wavedata");

  comm = app.communicator ();
  int nProcesses = comm.Get_size (); // how many processes are there?
  int rank = comm.Get_rank ();       // which process am I?
  int width = wave_port->width ();

  // For clarity, assume that width is a multiple of n_processes
  int nLocalVars = width / nProcesses;
  data = new double[nLocalVars];
  for (int i = 0; i < nLocalVars; ++i)
    data[i] = 0.0;

  // Declare what data we have to export
  MUSIC::ArrayData dmap (data,
			 MPI::DOUBLE,
			 rank * nLocalVars,
			 nLocalVars);
  wave_port->map (&dmap);

  double stoptime;
  app.config ("stoptime", &stoptime);
  app.enterSimulationLoop (TIMESTEP);

  std::cout << "Producer entering actual loop with ticks" << std::endl;
  for (; app.time () < stoptime; app.tick ())
    {
      if (rank == 0)
		{
		  // Generate original data on master node
		  int i;
		  double time = app.time ();

		  for (i = 0; i < nLocalVars; ++i)
			data[i] = sin (2 * M_PI * time * i);
		}

      // Broadcast these data out to all nodes
	  /* if (nProcesses > 1) */
		  /* comm.Bcast (data, nLocalVars, MPI::DOUBLE, 0); */
    }

  std::cout << "Finalizing" << std::endl;
  app.finalize ();

  return 0;
}
