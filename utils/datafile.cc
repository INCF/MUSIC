/* This file is part of the skol suite.
   Copyright (C) 2005, 2008 Mikael Djurfeldt

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


#include <iostream>
#include <limits>
#include <cctype>

#include "datafile.h"

using std::numeric_limits;

bool
Datafile::atHeaderLine ()
{
  return peek () == '#';
}

void
Datafile::ignoreLine ()
{
  ignore (numeric_limits<int>::max (), '\n');
}

void
Datafile::ignoreWhitespace ()
{
  while (isspace (peek ()))
    ignore ();
}

bool
Datafile::scanLine (const char* pattern)
{
  const char* p = pattern;
  while (*p != '\0')
    {
      if (peek () == '\n')
	{
	  ignore ();
	  return false;
	}
      if (peek () == *p)
	++p;
      else
	p = pattern;
      ignore ();
    }
  return true;
}

bool
Datafile::read (const char* pattern, int& x)
{
  seekg (0);
  while (atHeaderLine ())
    if (scanLine (pattern))
      {
	*this >> x;
	return true;
      }
  return false;
}

bool
Datafile::read (const char* pattern, double& x)
{
  seekg (0);
  while (atHeaderLine ())
    if (scanLine (pattern))
      {
	*this >> x;
	return true;
      }
  return false;
}

void
Datafile::skipHeader ()
{
  ignoreWhitespace ();
  while (atHeaderLine ())
    ignoreLine ();
}
