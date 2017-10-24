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

#ifndef MUSIC_CONFIGURATION_HH

#include "music/music-config.hh"

#include <string>
#include <map>

#include "music/application_map.hh"
#include "music/connectivity.hh"

namespace MUSIC {

  class Configuration {
  public:
      static const char* const configEnvVarName;
      typedef std::map<std::string, std::string> ConfigDict;
  private:

    std::string app_name_;

    ApplicationMap* applications_;
    Connectivity* connectivityMap_;

    Configuration* defaultConfig_;

    std::map<std::string, std::string> dict_;


  public:

    Configuration ();

#if MUSIC_USE_MPI
    Configuration (std::string configStr);
#endif

    ~Configuration ();

    void writeEnv ();

    bool lookup (std::string name);

    bool lookup (std::string name, int* result);

    bool lookup (std::string name, double* result);

    bool lookup(std::string name, std::string* result);

    bool lookup (std::string name, bool* result);

    void insert (std::string name, std::string value);

    const ConfigDict &getDict();

    void setDict(const ConfigDict &dict);

    void resetDict();

    std::string Name();

    void setName( std::string name);

    int Color ();

    int Leader ();

    ApplicationMap* applications ();

    Connectivity* connectivityMap ();

    Configuration* defaultConfig();

  private:

    void init ();

#if MUSIC_USE_MPI
    void parse (std::string configStr);
#endif

    void write (std::ostringstream& env, Configuration* mask);

  };

}

#define MUSIC_CONFIGURATION_HH
#endif
