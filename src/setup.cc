/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007, 2008, 2009, 2022 INCF
 *
 *  MUSIC is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  MUSIC is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "music/setup.hh"
#if MUSIC_USE_MPI
#include "music/mpi_utils.hh"
#include "music/runtime.hh"
#include "music/parse.hh"
#include "music/error.hh"
#include "music/application_mapper.hh"
#include "music/ioutils.hh"
#include <strings.h>
#include <fstream>

namespace MUSIC {

  bool Setup::isInstantiated_ = false;
  static std::string err_MPI_Init = "MPI_Init was called before the Setup constructor";
  const char* const Setup::opConfigFileName = "--music-config";
  const char* const Setup::opAppLabel = "--app-label";

  Setup::Setup (int& argc, char**& argv)
    : argc_ (argc), argv_ (argv)
  {
    checkInstantiatedOnce (isInstantiated_, "Setup");
    if (mpi_is_initialized ())
      errorRank (err_MPI_Init);
    maybeProcessMusicArgv (argc, argv);
    MPI_Init (&argc, &argv);

    init (argc, argv);

  }


  Setup::Setup (int& argc, char**& argv, int required, int* provided)
    : argc_ (argc), argv_ (argv)
  {
    checkInstantiatedOnce (isInstantiated_, "Setup");
    if (mpi_is_initialized ())
      errorRank (err_MPI_Init);
    maybeProcessMusicArgv (argc, argv);
    MPI_Init_thread (&argc, &argv, required, provided);
    init (argc, argv);
  }


  Setup::~Setup ()
    {
      for (std::vector<Port*>::iterator i = ports_.begin ();
           i != ports_.end ();
           ++i)
        (*i)->setupCleanup ();

      if (launchedByMusic ())
        delete temporalNegotiator_;

      // delete connection objects
      for (std::vector<Connection*>::iterator i = connections_->begin ();
           i != connections_->end ();
           ++i)
        delete *i;

      delete connections_;

      delete config_;

      isInstantiated_ = false;
    }


  void
  Setup::init (int& argc, char**& argv)
  {
    int myRank = mpi_get_rank (MPI_COMM_WORLD);
    std::string config = "";
    launchedByMusic_ = false;
    postponeSetup_ = false;

    if (launchedWithExec (config))
      {
        assert(config.length() > 0);
        launchedByMusic_ = true;
        if (!config.compare (0, 8, "POSTPONE"))
	  {
	    postponeSetup_ = true;
	    // *fixme* Error checking
	    std::istringstream in (config);
	    IOUtils::read (in); // POSTPONE
	    in.ignore (); // delim
	    std::string colorString = IOUtils::read (in);
	    color_ = atoi (colorString.c_str ());
	  }
	else
	  config_ = new Configuration (config);
      }
    else if (launchedMPMD (argc, argv, config))
      {
        launchedByMusic_ = true;
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
        app_mapper.map(&config_istream, binary, app_label);
      }
    else
      config_ = new Configuration ();


    connections_ = new std::vector<Connection*>; // destroyed by runtime
    if (launchedByMusic ())
      {
        // launched by the music utility
        if (!postponeSetup_)
          {
            fullInit ();
            argc = argc_;
            argv = argv_;
          }
        MPI_Comm_split (MPI_COMM_WORLD, postponeSetup_ ? color_ : config_->Color (), myRank, &comm);
      }
    else
      {
        // launched with mpirun
        comm = MPI_COMM_WORLD;
        timebase_ = MUSIC_DEFAULT_TIMEBASE;
      }
  }


  bool
  Setup::getOption (int argc, char** argv, std::string option, std::string& result)
  {
    result.assign("");
    for (int i = 1; i < argc; ++i)
       if (option.compare(argv[i]) == 0 && argc > i){
           result.assign(argv[i+1]);  // skip options
             return true;
       }
     return false;
  }


  bool
  Setup::launchedWithExec (std::string &result)
  {
    // is _MUSIC_CONFIG_ env variable is set ?
    char* res = getenv (Configuration::configEnvVarName);
    if (res != NULL)
      {
        result.assign (res);
        return true;
      }
    else
      return false;
  }


  bool
  Setup::launchedMPMD (int argc, char** argv, std::string& config)
  {
    // if given option --music-config,
    // the launch is categorized as MPMD
    if (!getOption(argc, argv, opConfigFileName, config) )
      return false;
    else
      return true;
  }


  void
  Setup::loadConfigFile (std::string filename, std::string &result)
  {
    std::ifstream config;
    char* buffer;
    int size = 0;
    int myRank = mpi_get_rank (MPI_COMM_WORLD);
    // Rank #0 is reading a file and broadcast it to each rank in the launch
    if (myRank == 0)
      {
        config.open (filename.c_str ());
        if (!config.is_open ())
          {
            std::ostringstream oss;
            oss << "MUSIC: Couldn't open configuration file: " << filename << '\n';
            error0 (oss.str ());
          }

        size = config.tellg ();
        config.seekg (0, std::ios_base::end);
        long cur_pos = config.tellg ();
        size = cur_pos - size;
        config.seekg (0, std::ios_base::beg);
      }
    // first broadcast the size of the file
    MPI_Bcast (&size, 1, MPI_INT, 0, MPI_COMM_WORLD);
    buffer = new char[size];

    if (myRank == 0)
      config.read (buffer, size);
    // then broadcast the file but itself
    MPI_Bcast (buffer, size, MPI_BYTE, 0, MPI_COMM_WORLD);
    // parseMapFile (app_name, std::string (buffer, size), result);
    if (myRank == 0)
      config.close ();

    result.assign (buffer);
    delete[] buffer;
  }


  void
  Setup::maybeProcessMusicArgv (int& argc, char**& argv)
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


  void
  Setup::maybePostponedSetup ()
  {
    if (postponeSetup_)
      {
	postponeSetup_ = false;
	std::string config = "";
	launchedWithExec (config);
	config_ = new Configuration (config);
	fullInit ();
      }
  }


  void
  Setup::errorChecks ()
  {
    ApplicationMap* apps = applicationMap ();
    int nRequestedProc = apps->nProcesses ();
    int nMPIProc = mpi_get_comm_size (MPI_COMM_WORLD);
    if (nMPIProc != nRequestedProc)
      {
	std::ostringstream msg;
	msg << "configuration file specifies " << nRequestedProc
	    << " MPI processes but MUSIC was given " << nMPIProc
	    << std::endl;
	error0 (msg.str ());
      }
  }


  void
  Setup::fullInit ()
  {
    errorChecks ();
    if (!config ("timebase", &timebase_))
      timebase_ = MUSIC_DEFAULT_TIMEBASE;	       // default timebase
    string binary;
    config_->lookup ("binary", &binary);
    string args;
    config_->lookup ("args", &args);
    argv_ = parseArgs (binary, args, &argc_);
    temporalNegotiator_ = new TemporalNegotiator (this);
  }
  

  bool
  Setup::launchedByMusic ()
  {
    return launchedByMusic_;
  }

  
  MPI_Comm
  Setup::communicator ()
  {
    return comm;
  }


  ConnectivityInfo*
  Setup::portConnectivity (const std::string localName)
  {
    return config_->connectivityMap ()->info (localName);
  }


  ApplicationMap*
  Setup::applicationMap ()
  {
    return config_->applications ();
  }


  int
  Setup::applicationColor ()
  {
	  return config_->Color();
  }


  std::string
  Setup::applicationName()
  {
    return config_->Name();
  }


  int
  Setup::leader ()
  {
    return config_->Leader ();
  }


  int
  Setup::nProcs ()
  {
    return mpi_get_comm_size (comm);
  }


  ConnectivityInfo::PortDirection
  Setup::portDirection (const std::string localName)
  {
    return config_->connectivityMap ()->direction (localName);
  }


  int
  Setup::portWidth (const std::string localName)
  {
    return config_->connectivityMap ()->width (localName);
  }


  PortConnectorInfo
  Setup::portConnections (const std::string localName)
  {
    return config_->connectivityMap ()->connections (localName);
  }


  bool
  Setup::config (string var, string* result)
  {
    return config_->lookup (var, result);
  }

  
  bool
  Setup::config (string var, int* result)
  {
    return config_->lookup (var, result);
  }

  
  bool
  Setup::config (string var, double* result)
  {
    return config_->lookup (var, result);
  }


  bool
  Setup::config (string var, bool* result)
  {
    return config_->lookup (var, result);
  }

  
  ContInputPort*
  Setup::publishContInput (std::string identifier)
  {
    return new ContInputPort (this, identifier);
  }


  ContOutputPort*
  Setup::publishContOutput (std::string identifier)
  {
    return new ContOutputPort (this, identifier);
  }


  EventInputPort*
  Setup::publishEventInput (std::string identifier)
  {
    return new EventInputPort (this, identifier);
  }


  EventOutputPort*
  Setup::publishEventOutput (std::string identifier)
  {
    return new EventOutputPort (this, identifier);
  }

  
  MessageInputPort*
  Setup::publishMessageInput (std::string identifier)
  {
    return new MessageInputPort (this, identifier);
  }


  MessageOutputPort*
  Setup::publishMessageOutput (std::string identifier)
  {
    return new MessageOutputPort (this, identifier);
  }

  
  void Setup::addPort (Port* p)
  {
    ports_.push_back (p);
  }

  
  void Setup::addConnection (Connection* c)
  {
    connections_->push_back (c);
  }

}
#endif
