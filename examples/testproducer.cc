#include <mpi.h>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <music.hh>

#define TIMESTEP 0.001

MPI::Intracomm comm;
double* data;
double* data2;

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
  data2 = new double[nLocalVars];
  for (int i = 0; i < nLocalVars; ++i)
    data[i] = 0.0;

  // Declare what data we have to export
  MUSIC::ArrayData dmap (data,
			 MPI::DOUBLE,
			 rank * nLocalVars,
			 nLocalVars);
  wave_port->map (&dmap);

  MUSIC::ArrayData dmap2 (data2,
			 MPI::DOUBLE,
			 rank * nLocalVars,
			 nLocalVars);

  double stoptime;
  app.config ("stoptime", &stoptime);

  std::ostringstream filename;
  filename << argv[1] << rank << ".producer.out";
  std::ofstream file (filename.str ().data ());

  // Define simulation loop
  auto loop = [&] (double stoptime)
  {
	  for (;app.time () < stoptime; app.tick ())
		{
		  if (rank == 0)
			{
			  for (int i = 0; i < nLocalVars; ++i)
				  file << data[i] << ' ';
			  file << std::endl;
			  // Generate original data on master node
			  int i;
			  double time = app.time ();

			  for (i = 0; i < nLocalVars; ++i)
				data[i] = sin (2 * M_PI * time * i);
			  std::cout << "Simtime: " << app.time () << std::endl;
			}

		  // Broadcast these data out to all nodes
		  if (nProcesses > 1)
			  comm.Bcast (data, nLocalVars, MPI::DOUBLE, 0);
		}
  };

  // First loop
  app.enterSimulationLoop (TIMESTEP);
  loop (stoptime);
  app.exitSimulationLoop ();

  // Fiddling around
  auto& portManager = app.getPortConnectivityManager ();
  portManager.connect ("consumer", "newPort", "producer", "newPort", 10, MUSIC::CommunicationType::POINTTOPOINT, MUSIC::ProcessingMethod::TREE);


  wave_port.reset ();
  wave_port = app.publish<MUSIC::ContOutputPort> ("wavedata");
  wave_port->map(&dmap);

  auto newPort = app.publish<MUSIC::ContInputPort> ("newPort");
  newPort->map(&dmap2, 1.);

  // Second loop
  app.enterSimulationLoop (TIMESTEP);
  loop (stoptime * 2.);


  std::cout << "Finalizing" << std::endl;
  app.finalize ();

  return 0;
}
