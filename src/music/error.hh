/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007, 2009 INCF
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

#ifndef MUSIC_ERROR_HH

#include <sstream>
#include <string>

namespace MUSIC {

  void error ();
  void hang ();
  void error (std::string msg);
  void error (std::ostringstream& ostr);
  void error0 (std::string msg);
  void errorRank (std::string msg);
  void checkOnce (bool& flag, std::string msg);
  void checkInstantiatedOnce (bool& flag, std::string className);
  void checkCalledOnce (bool& isCalled,
			std::string funcName,
			std::string suffix);
  
}

#define MUSIC_ERROR_HH
#endif
