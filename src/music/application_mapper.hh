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

#ifndef MUSIC_APPLICATION_MAPPER_HH

#include <istream>
#include <map>

#include <music/configuration.hh>
#include "config.h"

namespace rude {
  class Config;
}

namespace MUSIC {


  class ApplicationSelector
  {

  protected:
    bool selected_;

  public:
    ApplicationSelector (): selected_ (false)
    {
    }
    ;
    virtual
    ~ApplicationSelector ()
    {
    }
    ;
    virtual bool
    selected (Configuration* config) = 0;

    virtual void
    reset ()
    {
      selected_ = false;
    }
  };


  class SelectorINO : public ApplicationSelector
  {
    long ino_;
    int my_chunk_;
    int next_chunk_;
  public:
    SelectorINO (long ino, int index) :
      ApplicationSelector (), ino_ (ino), my_chunk_ (index), next_chunk_(0)
    {
    }
    ;
    bool
    selected (Configuration* config);

    void
    reset ()
    {
      ApplicationSelector::reset();
      next_chunk_ = 0;
    }
  };


  class SelectorOp : public ApplicationSelector
  {
    std::string app_label_;
  public:
    SelectorOp (std::string appLabel) :
        ApplicationSelector (), app_label_ (appLabel)
    {
    }
    ;
    bool
    selected (Configuration* config);

  };


  class SelectorR : public ApplicationSelector
  {
    int rank_;
    int next_chunk_;
  public:
    SelectorR (int rank) :
        ApplicationSelector (), rank_ (rank), next_chunk_(0)
    {
    }
    ;
    bool
    selected (Configuration* config);

  };


  class ApplicationMapper
  {

    rude::Config* cfile_;
    Configuration* config_;
    ApplicationMap* applications_;
    Connectivity* connectivityMap_;
    ApplicationSelector* app_selector_;

  public:
    ApplicationMapper (Configuration *config);

    ~ApplicationMapper();

    Configuration* map (std::istream* configFile, int rank);

#if MUSIC_USE_MPI
    Configuration* map (std::istream* configFile, std::string binary, std::string appLabel);
#endif

#if HAVE_SYS_STAT_H
    static long getApplicationINO (const char *path);
#endif

  private:

    void map(std::istream* configFile);

    void mapApplications ();

    void mapConnectivity (bool leaders);

    void mapSection(int index, Configuration* config);


  };

}

#define MUSIC_APPLICATION_MAPPER_HH
#endif
