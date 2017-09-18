#include "music/music_context.hh"

#if MUSIC_USE_MPI
#include <mpi.h>

#include <fstream>
#include <cassert>

#include "music/application_mapper.hh"
#include "music/parse.hh"
#include "music/misc.hh"
#include "music/error.hh"

namespace MUSIC
{
  const char* const OptionConstants::opConfigFileName = "--music-config";
  const char* const OptionConstants::opAppLabel = "--app-label";
  static std::string err_env_invalid = "The given environment variable is not set!";
  static std::string err_fac_functions_not_responsible = "Attempt to perform an action on unresponsible LaunchContext";



	void initialize_MPI(int& argc, char**& argv, int required, int* provided)
	{
		// TODO check if this if-block is symantically correct
		if (!MPI::Is_initialized ())
		{
			  /* errorRank (err_MPI_Init); */
#ifdef HAVE_CXX_MPI_INIT_THREAD
			*provided = MPI::Init_thread (argc, argv, required);
#else
			// Only C version provided in libmpich
			MPI_Init_thread (&argc, &argv, required, provided);
#endif
		}
	}

	void initialize_MPI(int& argc, char**& argv)
	{
		if (!MPI::Is_initialized ())
			MPI::Init (argc, argv);
	}




  std::unique_ptr<Configuration> MPMDContext::assembleConfiguration ()
  {
		std::string config_str = "";
        std::string config_file;
		OptionHelpers::getOption(argc_, argv_, OptionConstants::opConfigFileName, config_str);
        loadConfigFile (config_str, config_file);

        std::string app_label;
        std::string binary (argv_[0]);
        // argv[0] is the name of the program,
        // or an empty string if the name is not available
        if (!OptionHelpers::getOption (argc_, argv_, OptionConstants::opAppLabel, app_label)
            && binary.length () == 0)
          {
            std::ostringstream oss;
            oss << "MUSIC: use --app-label to specify application label";
            error0 (oss.str ());
          }

        std::istringstream config_istream (config_file);
        Configuration* config = new Configuration ();
        ApplicationMapper app_mapper(config);
        app_mapper.map(&config_istream, binary, app_label);
		return std::unique_ptr<Configuration> (config);
  }

  std::unique_ptr<Configuration> MPMDContext::getConfiguration ()
  {
	  return std::move (config_);
  }

  void MPMDContext::loadConfigFile (std::string filename, std::string &result)
  {
    std::ifstream config;
    char* buffer;
    int size = 0;
    int myRank = MPI::COMM_WORLD.Get_rank ();
    // Rank #0 is reading a file and broadcast it to each rank in the launch
    if (myRank == 0)
      {
        config.open (filename.c_str ());
        if (!config.is_open ())
          {
            std::ostringstream oss;
            oss << "MUSIC: Couldn't open configuration file: " << filename;
            error0 (oss.str ());
          }

        size = config.tellg ();
        config.seekg (0, std::ios_base::end);
        long cur_pos = config.tellg ();
        size = cur_pos - size;
        config.seekg (0, std::ios_base::beg);
      }
    // first broadcast the size of the file
    MPI::COMM_WORLD.Bcast (&size, 1, MPI::INT, 0);
    buffer = new char[size];

    if (myRank == 0)
      config.read (buffer, size);
    // then broadcast the file but itself
    MPI::COMM_WORLD.Bcast (buffer, size, MPI::BYTE, 0);
    // parseMapFile (app_name, std::string (buffer, size), result);
    if (myRank == 0)
      config.close ();

    result.assign (buffer);
    delete[] buffer;
  }



  MPI_Comm MPMDContext::getComm()
  {
    int myRank = MPI::COMM_WORLD.Get_rank ();
	return MPI::COMM_WORLD.Split (config_->Color (), myRank);
  }



  std::unique_ptr<Configuration> ExecContext::getConfiguration()
  {
	std::string config_str;
	config_str.assign(EnvHelpers::read_env(Configuration::configEnvVarName));
	assert(config_str.length() > 0);
	return std::make_unique<Configuration> (config_str);
  }

  MPI_Comm ExecContext::getComm()
  {
	return MPI::COMM_WORLD;
  }

  std::unique_ptr<Configuration> DefaultContext::getConfiguration()
  {
	return std::make_unique<Configuration> ();
  }

  MPI_Comm DefaultContext::getComm()
  {
	return MPI::COMM_WORLD;
  }

  std::unique_ptr<MusicContext> MusicContextFactory::createContextImpl(int argc, char** argv) const
  {
	for (auto& fac_function : fac_functions_)
	{
		std::unique_ptr<MusicContext> ptr {fac_function (argc, argv)};
		if (ptr != nullptr)
			return std::move (ptr);
	}
	return nullptr;
  }

  std::unique_ptr<MusicContext> MusicContextFactory::createContext(int& argc, char**& argv)
  {
	initialize_MPI (argc, argv);
	return std::unique_ptr<MusicContext> {createContextImpl (argc, argv)};
  }

  std::unique_ptr<MusicContext> MusicContextFactory::createContext (int& argc, char**& argv, int required, int* provided)
  {
	initialize_MPI (argc, argv, required, provided);
	return std::unique_ptr<MusicContext> {createContextImpl (argc, argv)};
  }

  void MusicContextFactory::addContextFactoryfunction (ContextFactoryfunction fac_function)
  {
	  fac_functions_.insert (fac_functions_.begin(), fac_function);
  }

  ContextFactoryfunction FactoryFunctions::MPMDContextFactory = [] (int argc, char** argv) -> MusicContext*
	{
		std::string config = "";
		if (!OptionHelpers::getOption(argc, argv, OptionConstants::opConfigFileName, config) )
		{
			return nullptr;
		}

		return new MPMDContext (argc, argv);
	};

	ContextFactoryfunction FactoryFunctions::ExecContextFactory = [] (int argc, char** argv) -> MusicContext*
	{
		if (EnvHelpers::read_env(Configuration::configEnvVarName) != NULL)
			return new ExecContext (argc, argv);
		else
			return nullptr;
	};

	ContextFactoryfunction FactoryFunctions::DefaultContextFactory = [] (int argc, char** argv) -> MusicContext*
	{
		return new DefaultContext ();
	};
}
#endif
