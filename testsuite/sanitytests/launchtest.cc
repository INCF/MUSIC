#include <music.hh>

#include <iostream>

int
main (int argc, char *argv[])
{
  auto context = MUSIC::MusicContextFactory ().createContext (argc, argv);
  MUSIC::Application app (std::move(context));

  MPI::Intracomm comm = app.communicator ();

  int rank = comm.Get_rank ();
  double param;
  if (!app.config ("param", &param))
    param = -1.0;
  std::cout << "rank=" << rank << ":param=" << param;
  for (int i = 0; i < argc; ++i)
    std::cout << ':' << argv[i];
  std::cout << std::endl;
  app.finalize ();
  return 0;
}
