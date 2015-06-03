/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2009 INCF
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

#ifndef MUSIC_CONNECTION_HH
#include "music/music-config.hh"
#if MUSIC_USE_MPI
#include <music/connector.hh>

namespace MUSIC {
  /*
   * The Connection class carries the extra information needed for
   * setting up connectors and subconnectors during spatial and
   * temporal negotiation.  Connection objects are deallocated at the
   * end of the Runtime constructor.
   */
  class Connection {
  protected:
    Connector* connector_;
    int maxBuffered_;
  public:

    Connection (Connector* connector, int maxBuffered)
      : connector_ (connector),
	maxBuffered_ (maxBuffered) { }
    virtual ~Connection () { } // needed to make base polymorphic (!)
    Connector* connector () const { return connector_; }
    //void setConnector (Connector* connector) { connector_ = connector; }
    int maxBuffered () { return maxBuffered_; }
  };
  
  class OutputConnection : public Connection {
  private:
    int elementSize_;
  public:
    OutputConnection (Connector* connector,
		      int maxBuffered,
		      int elementSize);
    int elementSize () { return elementSize_; }
  };
  
  class InputConnection : public Connection {
  private:
    ClockState accLatency_;
    bool interpolate_;
  public:
    InputConnection (Connector* connector,
		     int maxBuffered,
		     ClockState accLatency,
		     bool interpolate);
    ClockState accLatency () { return accLatency_; }
    bool interpolate () { return interpolate_; }
  };

}
#endif
#define MUSIC_CONNECTION_HH
#endif
