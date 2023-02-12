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

#include "music/music-config.hh"

#include "music/debug.hh"

#include "music/error.hh"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <algorithm>
#include <assert.h>
#include "music/application_mapper.hh"

#include "rudeconfig/src/config.h"

#if MUSIC_USE_MPI
  #include "music/mpi_utils.hh"
#endif

#if HAVE_SYS_STAT_H
  #include <sys/stat.h>
#endif

namespace MUSIC {


  // This is where we parse the configuration file
  // NOTE: Could check here that obligatory parameters exists

  ApplicationMapper::ApplicationMapper (Configuration *config)
  {
    config_ = config;
    applications_ = config->applications();
    connectivityMap_ = config->connectivityMap();
    cfile_ = new rude::Config ();
    app_selector_ = NULL;
  }


  ApplicationMapper::~ApplicationMapper()
  {
    if (app_selector_ != NULL)
      delete app_selector_;
    delete cfile_;
  }


  Configuration*
  ApplicationMapper::map (std::istream* configFile, int rank)
  {
    MUSIC_LOG("Co-simulation launched using music utility");
    app_selector_ = new SelectorR (rank);
    map (configFile);
    return config_;
  }


#if MUSIC_USE_MPI
  Configuration*
  ApplicationMapper::map (std::istream* configFile, std::string binary,
      std::string appLabel)
  {

    //how do we identify ourselves
    if (appLabel.length () != 0)
      {
        MUSIC_LOG("MPMD launch with --app-label option:" << appLabel);
        app_selector_ = new SelectorOp (appLabel);
      }
    else
      {
        int nRanks = mpi_get_comm_size (MPI_COMM_WORLD);
        int my_rank = mpi_get_rank (MPI_COMM_WORLD);

#if HAVE_SYS_STAT_H

        long my_ino = ApplicationMapper::getApplicationINO (binary.c_str ());
        // communicate st.st_ino
        long *inos = new long[nRanks];
        inos[my_rank] = my_ino;
        MPI_Allgather (MPI_IN_PLACE, 0, MPI_LONG, inos, 1,
		       MPI_LONG, MPI_COMM_WORLD);

        int index = 0;
        int i = 0;
        // among processors with the same ino number
        // assign current sequential order number.
        while (i < my_rank)
          if (inos[i++] == my_ino)
            index++;
        delete[] inos;

        MUSIC_LOG("MPMD launch. Application INO:" << my_ino);
        app_selector_ = new SelectorINO (my_ino, index);
#else
        std::ostringstream oss;
        oss << "MUSIC: HAVE_SYS_STAT_H is node defined, use --app-label to specify application label";
        error0 (oss.str ());
#endif

      }
    map (configFile);
    return config_;
  }
#endif

void
  ApplicationMapper::map (std::istream* configFile)
  {

    if (cfile_->load (*configFile))
      {
        bool leader_avail = false;
        mapApplications ();
#if MUSIC_USE_MPI
        if (mpi_is_initialized ())
          {
            applications_->assignLeaders (config_->Name ());
            leader_avail = true;
          }
#endif
        mapConnectivity (leader_avail);
      }
    else
      error0 ("Configuration file load: " + std::string (cfile_->getError ()));
  }


  void
  ApplicationMapper::mapApplications ()
  {

    // retrieve default configurations for current object config_
    mapSection ( 0, config_->defaultConfig());

    // create temporal object with the default configurations
    Configuration *config = new Configuration ();
    config->defaultConfig ()->setDict(config_->defaultConfig ()->getDict ());

    int n_selections = 0;
    int np;
    int nSections = cfile_->getNumSections ();
    // loop through all sections
    for (int s = 1; s < nSections; ++s)
      {
        // retrieve configuration to the temporal object
        mapSection (s, config);

        // if retrieved section is current configuration
        if (app_selector_->selected(config))
          {
            ++n_selections;
            // copy configuration settings to current object
            config_->setDict (config->getDict ());

            config_->setName(config->Name());

          }

        // save application information
        config->lookup ("np", &np);
        applications_->add (config->Name(), np, s - 1);
        // empty temporal object
        // NOTE: default configuration is preserved
        config->resetDict();
      }
    delete config;
    assert(config_->getDict().size() != 0);

    // NOTE: given rank -1 causes an assertion failure
    assert(n_selections == 1);
  }


  void
  ApplicationMapper::mapSection (int index, Configuration* config)
  {
    std::string name (cfile_->getSectionNameAt (index));
    cfile_->setSection (name.c_str ());
    int nMembers = cfile_->getNumDataMembers ();
    config->setName(name);
    for (int m = 0; m < nMembers; ++m)
      {
        std::string n (cfile_->getDataNameAt (m));
        std::string v (cfile_->getStringValue (n.c_str ()));
        config->insert (n,  v);
      }
  }


  void
  ApplicationMapper::mapConnectivity (bool leaders)
  {

    std::map<std::string, int> receiverPortCodes;
    int nextPortCode = 0;

    int nSections = cfile_->getNumSections ();
    for (int s = 0; s < nSections; ++s)
      {
        std::string secName (cfile_->getSectionNameAt (s));
        cfile_->setSection (secName.c_str ());

        int nConnections = cfile_->getNumSourceDestMembers ();
        for (int c = 0; c < nConnections; ++c)
          {
            std::string senderApp (cfile_->getSrcAppAt (c));
            std::string senderPort (cfile_->getSrcObjAt (c));
            std::string receiverApp (cfile_->getDestAppAt (c));
            std::string receiverPort (cfile_->getDestObjAt (c));
            std::string width (cfile_->getWidthAt (c));
            std::string commType (cfile_->getCommTypeAt (c));
            std::string procMethod (cfile_->getProcMethodAt (c));

            if (senderApp == "")
              {
                if (secName == "")
                  error (
                      "sender application not specified for output port "
                          + senderPort);
                else
                  senderApp = secName;
              }
            if (receiverApp == "")
              {
                if (secName == "")
                  error (
                      "receiver application not specified for input port "
                          + receiverPort);
                else
                  receiverApp = secName;
              }
            if (senderApp == receiverApp)
              error (
                  "port " + senderPort + " of application " + senderApp
                      + " connected to the same application");


            //  Communication type option can be either *point-to-point* or
            // *collective* written in any case letters.
            std::transform (commType.begin (), commType.end (),
                commType.begin (), ::tolower);
            if (commType.length () > 0 && commType.compare ("collective")
                && commType.compare ("point-to-point"))
              error ("communication type " + commType + " is not supported");

            //  Processing method can be either *tree* or
            // *table* written in any case letters.
            std::transform (procMethod.begin (), procMethod.end (),
                procMethod.begin (), ::tolower);
            if (procMethod.length () > 0 && procMethod.compare ("table")
                && procMethod.compare ("tree"))
              error ("processing method " + procMethod + " is not supported");

            // Generate a unique "port code" for each receiver port
            // name.  This will later be used during temporal
            // negotiation since it easier to communicate integers,
            // which have constant size, than strings.
            //
            // NOTE: This code must be executed in the same order in
            // all MPI processes.
            std::string receiverPortFullName = receiverApp + "." + receiverPort;
            std::map<std::string, int>::iterator pos = receiverPortCodes.find (
                receiverPortFullName);
            int portCode;
            if (pos == receiverPortCodes.end ())
              {
                portCode = nextPortCode++;
                receiverPortCodes.insert (
                    std::make_pair (receiverPortFullName, portCode));
              }
            else
              portCode = pos->second;

            ConnectivityInfo::PortDirection dir;
            ApplicationInfo* remoteInfo;
            if (config_->Name () == senderApp)
              {
                dir = ConnectivityInfo::OUTPUT;
                remoteInfo = applications_->lookup (receiverApp);
              }
            else if (config_->Name () == receiverApp)
              {
                dir = ConnectivityInfo::INPUT;
                remoteInfo = applications_->lookup (senderApp);
              }
            else
              continue;
            int w;
            if (width == "")
              w = ConnectivityInfo::NO_WIDTH;
            else
              {
                std::istringstream ws (width);
                if (! (ws >> w))
                  error ("could not interpret width");
              }

             // default communication type is *point-to-point*
            int iCommType;
            if (commType.length () == 0 || !commType.compare ("point-to-point"))
                iCommType = ConnectorInfo::POINTTOPOINT;
            else
                iCommType = ConnectorInfo::COLLECTIVE;

             // default processing method is *tree*
            int iProcMethod;
            if (procMethod.length () == 0 || !procMethod.compare ("tree"))
                iProcMethod = ConnectorInfo::TREE;

            else
                iProcMethod = ConnectorInfo::TABLE;

            // NOTE: leader number can be not available at this stage,
            // if leader is not available, we write down the color of the application
            int leader = leaders ? remoteInfo->leader () : remoteInfo->color ();
            connectivityMap_->add (
                dir == ConnectivityInfo::OUTPUT ? senderPort : receiverPort,
                dir, w, receiverApp, receiverPort, portCode, leader,
                remoteInfo->nProc (), iCommType, iProcMethod);
          }
      }
  }


#if HAVE_SYS_STAT_H
  long
  ApplicationMapper::getApplicationINO(const char * path)
    {
      struct stat st;
      if (stat(path, &st) == -1)
        {
          std::ostringstream oss;
          oss
          << "MUSIC: Couldn't retrieve file status (stat()) about: " << path;
          error0 (oss.str ());
        }
      return st.st_ino;
    }
#endif


  bool
  SelectorR::selected (Configuration* config)
  {
#if MUSIC_USE_MPI
    int np;
    config->lookup ("np", &np);
    selected_ = rank_ < next_chunk_;
    next_chunk_ += np;
    return rank_ < next_chunk_ && !selected_;
#endif

    return false;
  }


  bool
  SelectorOp::selected (Configuration* config)
  {
    std::string sec_name = config->Name();
    return !app_label_.compare(sec_name);
  }


  bool
  SelectorINO::selected (Configuration* config)
  {
#if HAVE_SYS_STAT_H
    std::string sec_name;
    config->lookup("appLabel", &sec_name);
    std::string binary;
    config->lookup("binary", &binary);
    long ino = ApplicationMapper::getApplicationINO (binary.c_str());
    if (ino == ino_)
      {
        int np;
        config->lookup("np", &np);
        selected_ = my_chunk_ < next_chunk_;
        next_chunk_+=np;
      }

    return ino == ino_ && my_chunk_ < next_chunk_ && !selected_;
#endif

    return false;

  }

}
