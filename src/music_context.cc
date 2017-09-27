#include "music/music_context.hh"

#if MUSIC_USE_MPI
#include <mpi.h>

#include <iostream>
#include <fstream>
#include <cassert>
#include <strings.h>
#include "music/application_mapper.hh"
#include "music/parse.hh"
#include "music/misc.hh"
#include "music/error.hh"

namespace MUSIC
{
  const char* const OptionConstants::opConfigFileName = "--music-config";
  const char* const OptionConstants::opAppLabel = "--app-label";
  static std::string err_MPI_Init = "MPI_Init was called before the Setup constructor";
  static std::string err_factory_no_handler = "No suitable factory method for the given argc and argv has been found";
  static std::string err_env_invalid = "The given environment variable is not set!";
  static std::string err_fac_functions_not_responsible = "Attempt to perform an action on unresponsible LaunchContext";

	  void
	  maybeProcessMusicArgv (int& argc, char**& argv)
	  {
		char* MUSIC_ARGV = getenv ("MUSIC_ARGV");
		if (MUSIC_ARGV != NULL)
		  {
		std::string cmd;
		std::string argstring;
		char* s = index (MUSIC_ARGV, ' ');
		if (s == NULL)
		  {
			cmd = std::string (MUSIC_ARGV);
			argstring = "";
		  }
		else
		  {
			cmd = std::string (MUSIC_ARGV, s - MUSIC_ARGV);
			argstring = std::string (s + 1);
		  }
		argv = parseArgs (cmd, argstring, &argc);
		  }
	  }


	void initialize_MPI(int& argc, char**& argv, int required, int* provided)
	{
		// TODO check if this if-block is symantically correct
		if (MPI::Is_initialized ())
			errorRank (err_MPI_Init);
			  /* errorRank (err_MPI_Init); */
		maybeProcessMusicArgv (argc, argv);
#ifdef HAVE_CXX_MPI_INIT_THREAD
		*provided = MPI::Init_thread (argc, argv, required);
#else
		// Only C version provided in libmpich
		MPI_Init_thread (&argc, &argv, required, provided);
#endif
	}

	void initialize_MPI(int& argc, char**& argv)
	{
		if (MPI::Is_initialized ())
			errorRank (err_MPI_Init);
		maybeProcessMusicArgv (argc, argv);
		MPI::Init (argc, argv);
	}




  std::unique_ptr<Configuration> MPMDContext::assembleConfigFromFile()
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



  MPI::Intracomm MPMDContext::getComm()
  {
    int myRank = MPI::COMM_WORLD.Get_rank ();
	std::cout << "Spawning comm_WORLD with color: " << config_->Color() << " and rank: " <<myRank << std::endl;
	return MPI::COMM_WORLD.Split (config_->Color (), myRank);
  }


  std::unique_ptr<Configuration> ExecContext::assembleConfigFromEnv ()
  {
	std::string config_str;
	config_str.assign(EnvHelpers::read_env(Configuration::configEnvVarName));
	assert(config_str.length() > 0);
	return std::unique_ptr<Configuration> (new Configuration (config_str));
  }

  std::unique_ptr<Configuration> ExecContext::getConfiguration()
  {
	return std::move(config_);
  }

  MPI::Intracomm ExecContext::getComm()
  {
    int myRank = MPI::COMM_WORLD.Get_rank ();
    return MPI::COMM_WORLD.Split (config_->Color (), myRank);
  }

  std::unique_ptr<Configuration> DefaultContext::getConfiguration()
  {
	return std::unique_ptr<Configuration> (new Configuration ());
  }

  MPI::Intracomm DefaultContext::getComm()
  {
	return MPI::COMM_WORLD;
  }

  std::unique_ptr<MusicContext> MusicContextFactory::createContextImpl(int argc, char** argv) const
  {
	for (auto& fac_function : fac_functions_)
	{
		std::unique_ptr<MusicContext> ptr {fac_function (argc, argv)};
		if (ptr != nullptr)
			return ptr;
	}
	errorRank (err_factory_no_handler);
  }

  std::unique_ptr<MusicContext> MusicContextFactory::createContext(int& argc, char**& argv)
  {
	initialize_MPI (argc, argv);
	auto context = createContextImpl (argc, argv);
	assert (context->getComm ()!= MPI::COMM_NULL);
	return context;
  }

  std::unique_ptr<MusicContext> MusicContextFactory::createContext (int& argc, char**& argv, int required, int* provided)
  {
	initialize_MPI (argc, argv, required, provided);
	auto context = createContextImpl (argc, argv);
	assert (context->getComm ()!= MPI::COMM_NULL);
	return context;
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

		std::cout << "Using MUSIC MPMDContext" << std::endl;
		return new MPMDContext (argc, argv);
	};

	ContextFactoryfunction FactoryFunctions::ExecContextFactory = [] (int argc, char** argv) -> MusicContext*
	{
		if (EnvHelpers::read_env(Configuration::configEnvVarName) != NULL)
		{
			std::cout << "Using MUSIC ExecContext" << std::endl;
			return new ExecContext (argc, argv);
		}
		else
			return nullptr;
	};

	ContextFactoryfunction FactoryFunctions::DefaultContextFactory = [] (int argc, char** argv) -> MusicContext*
	{
		std::cout << "Using MUSIC DefaultContext" << std::endl;
		return new DefaultContext ();
	};
}
#endif
