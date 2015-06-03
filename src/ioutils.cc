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

#include <cstdio>

#include "music/ioutils.hh"

namespace MUSIC {

  namespace IOUtils {
    
    void
    write (std::ostringstream& out, std::string s)
    {
      std::istringstream value (s);
      while (true)
	{
	  int c;
	  switch (c = value.get ())
	    {
	    case '\\':
	    case ':':
	      out << '\\';
	    default:
	      out << (char) c;
	      continue;
	    case EOF:
	      break;
	    }
	  break;
	}
    }

  
    std::string
    read (std::istringstream& in, int delim)
    {
      std::ostringstream value;
      while (true)
	{
	  int c = in.peek ();
	  switch (c)
	    {
	    case '\\':
	      in.get ();
	      value << (char) in.get ();
	      continue;
	    default:
	      if (c == delim)
		break;
	      value << (char) in.get ();
	      continue;
	    case EOF:
	      break;
	    }
	  break;
	}
      return value.str ();
    }

  
    std::string
    read (std::istringstream& in)
    {
      return read (in, ':');
    }

  }
  
}
