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

// viewevents.cpp written by Johannes Hjorth, hjorth@nada.kth.se


#include "VisualiseNeurons.h"
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char **argv) {

  VisualiseNeurons *vn = new VisualiseNeurons();
  vn->run(argc,argv);

  std::cout << "Done." << std::endl;

  vn->finalize();

}
