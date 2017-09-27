#include <mpi.h>
#include <music.hh>
#include <fstream>
#include <sstream>
#include <cmath>

#define TIMESTEP 0.0005

MPI::Intracomm comm;
double* data;
double* data2;

int
main (int argc, char* argv[])
{
  auto context = MUSIC::MusicContextFactory ().createContext (argc, argv);
  MUSIC::Application app (std::move(context));

  auto wavePort = app.publish<MUSIC::ContInputPort> ("wavedata");

  comm = app.communicator ();
  int nProcesses = comm.Get_size (); // how many processes are there?
  int rank = comm.Get_rank ();       // which process am I?
  int width = 0;
  if (wavePort->hasWidth ())
    width = wavePort->width ();
  else
    MPI::COMM_WORLD.Abort (1);

  std::cout << "Cosumer says width=" << width << std::endl;

  // For clarity, assume that width is a multiple of n_processes
  int nLocalVars = width / nProcesses;
  data = new double[nLocalVars];
  std::ostringstream filename;
  filename << argv[1] << rank << ".consumer.out";
  std::cout << "filename "  << argv[1] << std::endl;
  std::ofstream file (filename.str ().data ());

  // Init data2
  data2 = new double[nLocalVars];
  for (int i = 0; i < nLocalVars; ++i)
    data2[i] = 0.0;

  // Declare where in memory to put data
  MUSIC::ArrayData dmap (data,
			 MPI::DOUBLE,
			 rank * nLocalVars,
			 nLocalVars);
  wavePort->map (&dmap);

  MUSIC::ArrayData dmap2 (data2,
			 MPI::DOUBLE,
			 rank * nLocalVars,
			 nLocalVars);

  double stoptime;
  app.config ("stoptime", &stoptime);

  // Define simulation loop
  auto loop = [&] (double stoptime)
  {
	  for (; app.time () < stoptime; app.tick())
		{
		  // Dump to file
		  for (int i = 0; i < nLocalVars; ++i)
			  file << data[i] << ' ';
		  file << std::endl;

		  int i;
		  double time = app.time ();
		  for (i = 0; i < nLocalVars; ++i)
			data2[i] = sin (2 * M_PI * time * i);
		}
  };
  // First loop
  app.enterSimulationLoop (TIMESTEP);
  loop (stoptime);
  app.exitSimulationLoop ();

  // Fiddling around
  auto& portManager = app.getPortConnectivityManager ();
  portManager.connect ("consumer", "newPort", "producer", "newPort", 10, MUSIC::CommunicationType::POINTTOPOINT, MUSIC::ProcessingMethod::TREE);

  wavePort.reset () ;
  wavePort = app.publish<MUSIC::ContInputPort> ("wavedata");
  wavePort->map (&dmap, 1.);

  auto newPort = app.publish<MUSIC::ContOutputPort> ("newPort");
  newPort->map(&dmap2);


  // Second loop
  app.enterSimulationLoop (TIMESTEP);
  loop (stoptime * 2.);
  std::cout << "Finalizing" << std::endl;
  app.finalize ();

  return 0;
}
