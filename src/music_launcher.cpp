#include <fstream>
#include <cassert>
#include "music/music_launcher.hh"
#include "music/application_mapper.hh"
#include "music/parse.hh"
#include "music/misc.hh"
#include "music/error.hh"

namespace MUSIC
{
  const char* const opConfigFileName = "--music-config";
  const char* const opAppLabel = "--app-label";
  static std::string err_env_invalid = "The given environment variable is not set!";
  static std::string err_handler_not_responsible = "Attempt to perform an action on unresponsible LaunchLauncher";
  char* read_env(std::string envVarName)
  {
	  return std::getenv(envVarName.c_str());
  }

  bool MPMDLauncher::isResponsible (int argc, char** argv)
  {
    std::string config = "";
    if (!getOption(argc, argv, opConfigFileName, config) )
      return false;
    else
      return true;
  }

  std::unique_ptr<Configuration> MPMDLauncher::assembleConfiguration ()
  {
		std::string config_str = "";
        std::string config_file;
		getOption(argc_, argv_, opConfigFileName, config_str);
        loadConfigFile (config_str, config_file);

        std::string app_label;
        std::string binary (argv_[0]);
        // argv[0] is the name of the program,
        // or an empty string if the name is not available
        if (!getOption (argc_, argv_, opAppLabel, app_label)
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

  std::unique_ptr<Configuration> MPMDLauncher::getConfiguration ()
  {
	  return std::move (config_);
  }

  void MPMDLauncher::loadConfigFile (std::string filename, std::string &result)
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

  bool
  MPMDLauncher::getOption (int argc, char** argv, std::string option, std::string& result)
  {
    result.assign("");
    for (int i = 1; i < argc; ++i)
       if (option.compare(argv[i]) == 0 && argc > i){
           result.assign(argv[i+1]);  // skip options
             return true;
       }
     return false;
  }


  MPI_Comm MPMDLauncher::getComm()
  {
    int myRank = MPI::COMM_WORLD.Get_rank ();
	return MPI::COMM_WORLD.Split (config_->Color (), myRank);
  }


  bool ExecLauncher::isResponsible()
  {
    // is _MUSIC_CONFIG_ env variable is set ?
	if (read_env(Configuration::configEnvVarName) != NULL)
		return true;
	return false;
  }

  std::unique_ptr<Configuration> ExecLauncher::getConfiguration()
  {
	std::string config_str;
	config_str.assign(read_env(Configuration::configEnvVarName));
	assert(config_str.length() > 0);
	return std::make_unique<Configuration> (config_str);
  }

  MPI_Comm ExecLauncher::getComm()
  {
	return MPI::COMM_WORLD;
  }

  std::unique_ptr<Configuration> DefaultLauncher::getConfiguration()
  {
	return std::make_unique<Configuration> ();
  }

  MPI_Comm DefaultLauncher::getComm()
  {
	return MPI::COMM_WORLD;
  }

  std::unique_ptr<MusicLauncher> MusicLauncherFactory::create(int argc, char** argv) const
  {
	for (auto& h : handler_)
	{
		if ((std::unique_ptr ptr = h (argc, argv)) != nullptr)
			return std::move (ptr);
	}
	return nullptr;
  }

  void MusicLauncherFactory::addHandler(LauncherSelectionHandler handler)
  {
	  launchers_.insert(launchers_.begin(), handler);
  }

}

