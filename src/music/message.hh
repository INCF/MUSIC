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

#ifndef MUSIC_MESSAGE_HH
#include <music/index_map.hh>

namespace MUSIC {

  class MessageHeader {
  public:
    typedef struct {
      double t;
      int size;
    } header_t;

  private:
    union {
      header_t header;
      char data[sizeof (header_t)];
    } u;
    
  public:
    MessageHeader (double t, int size)
    {
      u.header.t = t;
      u.header.size = size;
    }

    double t () { return u.header.t; }
    double size () { return u.header.size; }
    void* data () { return u.data; }
  };

  class MessageHandler {
  public:
    virtual void operator () (double t, void* msg, size_t size) = 0;
    virtual ~MessageHandler() {}
  };
  
  class MessageHandlerDummy : public MessageHandler {
  public:
    virtual void operator () (double t, void* msg, size_t size) { };
  };
  
  class MessageHandlerProxy
    : public MessageHandler {
    void (*messageHandler) (double t, void* msg, size_t size);
  public:
    MessageHandlerProxy () { }
    MessageHandlerProxy (void (*mh) (double t, void* msg, size_t size))
      : messageHandler (mh) { }
    void operator () (double t, void* msg, size_t size)
    {
      messageHandler (t, msg, size);
    }
  };

}
#define MUSIC_MESSAGE_HH
#endif
