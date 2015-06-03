/* This file is part of the skol suite.
   Copyright (C) 2005, 2008, 2009 Mikael Djurfeldt

   The skol suite is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation; either version 3, or (at
   your option) any later version.

   The skol suite is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with the skol suite; see the file COPYING.  If not, write to
   the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.  */


#ifndef DATAFILE_H

#include <string>
#include <fstream>

class Datafile : public std::ifstream {
 protected:
  bool atHeaderLine ();
  bool scanLine (const char* pattern);
 public:
  Datafile (std::string filename) : std::ifstream (filename.data ()) { }
  void ignoreLine ();
  void ignoreWhitespace ();
  bool read (const char*, int& x);
  bool read (const char*, double& x);
  void skipHeader ();
};
#define DATAFILE_H
#endif
