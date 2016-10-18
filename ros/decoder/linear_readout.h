/* -*- C++ -*-
 *  This file is part of MUSIC.
 *  Copyright (C) 2016 INCF
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

#include <music.hh>
#include <mpi.h>

#include <queue>
#include <cmath>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include "sys/time.h"
#include "jsoncpp/json/json.h"

#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_vector_double.h>
#include <gsl/gsl_blas.h>

#include <iostream>

#define DEBUG_OUTPUT false 

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_TAU = 0.03;
const string DEFAULT_WEIGHTS_FILENAME = "readout_weights.dat";


class LinearReadoutDecoder : MUSIC::EventHandlerGlobalIndex{
public:
  void init(int argc, char** argv);
  void runMUSIC();
  void finalize();

private:
  class Event {
  public:
    double t;
    int id;
    Event (double t_, int id_) : t (t_), id (id_) { }
    bool operator< (const Event& other) const { return t > other.t; }
  };
	
  MPI::Intracomm comm;
  MUSIC::Runtime* runtime;
  double stoptime;
  double timestep;
  double acceptable_latency;
  int size_command_data;
  int size_spike_data;
  double* command_data;
  gsl_vector_view vec_command_data;
  double* activity_traces;
  gsl_vector_view vec_activity_traces;
  unsigned int num_spikes0;

  string weights_filename;
  Json::Value json_readout_weights; 
  double* readout_weights;
  gsl_matrix_view mat_readout_weights;


  MUSIC::EventInputPort* port_in;
  MUSIC::ContOutputPort* port_out;

  double tau, inv_tau, propagator;
  std::priority_queue<Event> spikes;

  void initMUSIC(int argc, char** argv);
  void readWeightsFile();
  void operator() (double t, MUSIC::GlobalIndex id );
};


