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

#include "music/parse.hh"

#include <sstream>
#include <vector>
#include <cstdio>

namespace MUSIC {
  
  Parser::Parser (std::string s)
    : in (new std::istringstream (s))
  {
  
  }
  Parser::~Parser ()
  {
	  delete in;
  }
  
  void
  Parser::ignoreWhitespace ()
  {
    while (isspace (in->peek ()))
      in->ignore ();
  }

  
  void
  Parser::parseString (std::ostringstream& arg, char delim)
  {
    while (true)
      {
	int c;
	switch (c = in->get ())
	  {
	  case '\'':
	  case '"':
	    if (c == delim)
	      break;
	  default:
	    arg << (char) c;
	    continue;
	  case '\\':
	    arg << (char) in->get ();
	    continue;
	  case EOF:
	    // NOTE: generate error message
	    break;
	  }
	break;
      }  
  }

  
  std::string
  Parser::nextArg ()
  {
    std::ostringstream arg;
    while (true)
      {
	int c;
	switch (c = in->get ())
	  {
	  default:
	    arg << (char) c;
	    continue;
	  case '\\':
	    arg << (char) in->get ();
	    continue;
	  case '\'':
	  case '"':
	    parseString (arg, c);
	    continue;
	  case ' ':
	  case '\t':
	  case EOF:
	    break;
	  }
	break;
      }
    return arg.str ();
  }

  
  char **
  parseArgs (std::string cmd,
	     std::string argstring,
	     int* argc)
  {
    Parser in (argstring);
    std::vector<std::string> args;
    args.push_back (cmd);
    in.ignoreWhitespace ();

    while (! in.eof ())
      args.push_back (in.nextArg ());

    int nArgs = args.size ();
    char** result = new char*[nArgs + 1];
    for (int i = 0; i < nArgs; ++i)
      {
	int len = args[i].length ();
	result[i] = new char[len + 1];
	args[i].copy (result[i], len);
	result[i][len] = '\0';
      }
    result[nArgs] = 0;
    *argc = nArgs;
    return result;
  }

}
