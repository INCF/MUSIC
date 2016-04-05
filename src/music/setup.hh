/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007, 2008, 2009 INCF
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

#ifndef MUSIC_SETUP_HH
#include "music/music-config.hh"
#if MUSIC_USE_MPI
#include <mpi.h>

#include <string>
#include <vector>

#include <music/port.hh>

#include <music/index_map.hh>
#include <music/linear_index.hh>
#include <music/cont_data.hh>
#include <music/connector.hh>
#include <music/temporal.hh>
#include <music/configuration.hh>


using std::string;

#define MUSIC_DEFAULT_TIMEBASE 1e-9

namespace MUSIC {

  class Runtime;

  class Setup;

  struct GlobalSetupData
  {
    GlobalSetupData()
        : setups_(),
        config_(NULL),
        temporalNegotiator_(NULL),
        argc_(),
        argv_(NULL),
        launchedByMusic_(false),
        postponeSetup_(false),
        timebase_(0.0)
      {
      }


  ~GlobalSetupData()
  {
      setups_.clear();
      if (launchedByMusic_ )
      {
          delete temporalNegotiator_;
      }

      delete config_;
      delete argv_;
  }



    void remove_setup(Setup* s)
    {
        std::vector<Setup*>::iterator it = std::find(setups_.begin(), setups_.end(), s);
        if(it != setups_.end())
        {
            setups_.erase(it);
        }
    }

    void add_setup(Setup* s)
    {
        setups_.push_back(s);
    }

    std::vector<Setup*> setups_;
    Configuration* config_;
    TemporalNegotiator* temporalNegotiator_;
    int argc_;
    char** argv_;
    bool launchedByMusic_;
    bool postponeSetup_;
    double timebase_;
  };

  /*
   * This is the Setup object in the MUSIC API
   *
   * It is documented in section 4.3 of the MUSIC manual
   */
  
  class Setup {
    static const char* const configEnvVarName;
    static const char* const opConfigFileName;
    static const char* const opAppLabel;


  public:

    Setup (int& argc, char**& argv);

    Setup (int& argc, char**& argv, int required, int* provided);

    ~Setup ();

    MPI::Intracomm communicator ();

    bool config (string var, string* result);

    bool config (string var, int* result);

    bool config (string var, double* result);

    bool isLastSetupInstance();

    void clearPorts();

    void clearConnections();

    ContInputPort* publishContInput (string identifier);

    ContOutputPort* publishContOutput (string identifier);

    EventInputPort* publishEventInput (string identifier);

    EventOutputPort* publishEventOutput (string identifier);

    MessageInputPort* publishMessageInput (string identifier);

    MessageOutputPort* publishMessageOutput (string identifier);

  private:
    static GlobalSetupData data_;
    static MPI::Intracomm comm_;
    std::vector<Port*> ports_;
    std::vector<Connection*> connections_;
    // Since we don't want to expose this internal interface to the
    // user we put the member functions in the private part and give
    // these classes access through a friend declaration.  Classes are
    // still expected not to access the internal data members
    // directly.  A cleaner solution would be to split this class into
    // one user API part and one internal interface part.
    friend class Runtime;
    friend class Port;
    friend class OutputPort;
    friend class InputPort;
    friend class TemporalNegotiator;
    friend class ApplicationNode;
    
    static std::vector<Setup*> get_all_setups()
    {
        return std::vector<Setup*>( data_.setups_ );        
    }

    double timebase () { return data_.timebase_; }

    bool launchedByMusic ();

    void init (int& argc, char**& argv);

    void maybeProcessMusicArgv (int& argc, char**& argv);

    void maybePostponedSetup ();

    void fullInit ();

    ConnectivityInfo* portConnectivity (const std::string localName);

    ApplicationMap* applicationMap ();

    int applicationColor();

    std::string applicationName();

    int leader ();

    int nProcs ();

    ConnectivityInfo::PortDirection
    portDirection (const std::string localName);

    int portWidth (const std::string localName);

    PortConnectorInfo portConnections (const std::string localName);

    std::vector<Port*> ports ()
    {
      return ports_;
    }    

    void addPort (Port* p);
    
    std::vector<Connection*> connections ()
    {
      return connections_;
    }
    
    void addConnection (Connection* c);

    TemporalNegotiator* temporalNegotiator () { return data_.temporalNegotiator_; }
    
    void errorChecks ();

    bool launchedWithExec (std::string &configStr);

    bool launchedMPMD (int argc, char** argv, std::string& config);

    void loadConfigFile (std::string filename, std::string &result);

    bool getOption (int argc, char** argv, std::string option, std::string& result);

  };
  
}
#endif
#define MUSIC_SETUP_HH
#endif
