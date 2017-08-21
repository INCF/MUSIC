#include "music/music_launcher.hh"

namespace MUSIC
{
  const char* const MPMDLauncher::opConfigFileName = "--music-config";
  const char* const ConfigurationFactory::opAppLabel = "--app-label";
  static std::string err_env_invalid = "The given environment variable is not set!;
  static std::string err_handler_not_responsible = "Attempt to perform an action on unresponsible LaunchLauncher";

  bool MPMDLauncher::isResponsible()
  {
    std::string config = "";
    if (!getOption(argc_, argv_, opConfigFileName, config) )
      return false;
    else
      return true;
  }

  std::unique_ptr<Configuration> MPMDLauncher::getConfiguration()
  {
        std::string config_file;
        loadConfigFile (config, config_file);

        std::string app_label;
        std::string binary (argv[0]);
        // argv[0] is the name of the program,
        // or an empty string if the name is not available
        if (!getOption (argc, argv, opAppLabel, app_label)
            && binary.length () == 0)
          {
            std::ostringstream oss;
            oss << "MUSIC: use --app-label to specify application label";
            error0 (oss.str ());
          }

        std::istringstream config_istream (config_file);
        config_ = new Configuration ();
        ApplicationMapper app_mapper(config_);
		// TODO get PortCodes
        app_mapper.map(&config_istream, binary, app_label);
		return std::unique_ptr<Configuration> (config_);
  }

  void MPMDLauncher::loadConfigFile (std::string filename, std::string &result)
  {
	if (!isResponsible())
		error(err_handler_not_responsible);

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


  MPI::MPI_Comm MPMDLauncher::getComm()
  {
    int myRank = MPI::COMM_WORLD.Get_rank ();
	if (!isResponsible())
		error(err_handler_not_responsible);
	return MPI::COMM_WORLD.Split (config_->Color (), myRank);
  }

  char* ExecLauncher::read_env(std::string envVarName)
  {
	  return getenv(envVarName);
  }

  bool ExecLauncher::isResponsible()
  {
    // is _MUSIC_CONFIG_ env variable is set ?
    char* res = read_env (Configuration::configEnvVarName);
	if (read_env(Configuration::configEnvVarName) != NULL)
		return true;
	return false;
  }

  std::unique_ptr<Configuration> ExecLauncher::getConfiguration()
  {
	if (!isResponsible())
		error(err_handler_not_responsible);
	std::string config_str;
	config_str.assign(read_env(Configuration::configEnvVarName));
	assert(config_str.length() > 0);
	return std::make_unique<Configuration> (config_str);
  }

  MPI::MPI_Comm ExecLauncher::getComm()
  {
	if (!isResponsible())
		error(err_handler_not_responsible);
	return MPI::COMM_WORLD;
  }

  bool DefaultLauncher::isResponsible()
  {
	return true;
  }

  std::unique_ptr<Configuration> DefaultLauncher::getConfiguration()
  {
	if (!isResponsible())
		error(err_handler_not_responsible);
	return std::make_unique<Configuration> ();
  }

  MPI::MPI_Comm DefaultLauncher::getComm()
  {
	if (!isResponsible())
		error(err_handler_not_responsible);
	return MPI::COMM_WORLD;
  }

  MusicLauncher MUSICLauncherFactory::create(int argc, char** argv, MPI::MPI_Comm comm)
  {
	for (auto& e : launchers_)
	{
		e.init (argc, argv, comm);
		if (e.isResponsible())
			return e;

	}
	// Default case
	MusicLauncher default_launcher =  DefaultLauncher();
	default_launcher.init(argc, argv);
	return default_launcher;
  }

  void MUSICLauncherFactory::addLauncher(MusicLauncher s)
  {
	  launchers_.insert(launchers_.begin(), s);
  }

}

