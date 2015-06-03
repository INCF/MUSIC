/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2008, 2009 INCF
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

#ifndef MUSIC_APPLICATION_MAP_HH
#include "music/music-config.hh"
#include <sstream>
#include <vector>
#include <map>
namespace MUSIC {

  class ApplicationInfo
  {
    std::string name_;
    int nProc_;
    int color_; //used for convenience
    int leader_;

  public:
    ApplicationInfo (std::string name, int n, int c) :
        name_ (name), nProc_ (n), color_ (c), leader_(-1)
    {
    }


    std::string
    name ()
    {
      return name_;
    }


    int
    color ()
    {
      return color_;
    }


    int
    nProc ()
    {
      return nProc_;
    }


    int
    leader ()
    {
      return leader_;
    }


    void
    setLeader( int leader)
    {
      leader_ = leader;
    }
  };


  class ApplicationMap : public std::vector<ApplicationInfo>
  {

  public:
    ApplicationMap ();

    ApplicationInfo* lookup (std::string appName);

    ApplicationInfo* lookup (int color);

    int nProcesses ();

    void add (std::string name, int n, int c);

    void write (std::ostringstream& out);

    void read (std::istringstream& in);

#if MUSIC_USE_MPI
    std::map<int, int>
    assignLeaders (std::string my_app_label);
#endif
  };

}

#define MUSIC_APPLICATION_MAP_HH
#endif
