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

#include <iostream>
#include "stdlib.h" 
#include <math.h>
#include <vector>
#include <numeric>

class IAFNeuron{
  double tau_m, V_th, V_reset, I_e, C_m, R, resolution;
  double t_ref;// refactory time in ms 
  int is_ref; // refactory time in steps 
  double propagator;

  // NEF parameters
  std::vector<double> alpha;
  bool is_nef_initialized;

public:
  double  bias, V_m, scaling;
  IAFNeuron(int dimensions){
        
    //V_m = 0.0;
    V_m = std::rand() % 10;
    C_m = 250.0;
    tau_m = 20.0; 
    V_th = 10.0;
    V_reset = 0.0;
    I_e = 0.0;
    t_ref = 2;

    // dependent variables
    R = tau_m / C_m;
            
    is_ref = 0.;
    resolution = 0.0;

    // nef
    init_nef(dimensions);

  } 

  void setResolution(double resolution){
    this->resolution = resolution * 1000.;
    propagator = std::exp(-(this->resolution)/tau_m);
  }

  void encode(std::vector<double> data){
    I_e = std::inner_product(alpha.begin(), alpha.end(), data.begin(), bias);

  }

  bool propagate(){

    if (propagator == 0){
      std::cerr << "Resolution not set. Call setResolution() first" << std::endl;
      return false; 
    }
    bool emit_spike = false;

            
    if (is_ref == 0){
      //V_m += ((-(V_m-V_reset) + R * I_e) / tau_m ) * t; // forward euler?
      V_m = R * (1 - propagator) * I_e + propagator * V_m; // exact!

    }
    else{
      --is_ref;
    }
            
    if (V_m > V_th){
      V_m = V_reset;
      is_ref = int(t_ref/resolution);
      emit_spike = true;
    }
    return emit_spike;
           
  }

private:
  void init_nef(int dimensions){
    std::vector<double> pref_direction;
    double ssum = 0;
    scaling = 0. + std::rand() % 300;
    for (int i = 0; i < dimensions; ++i){
      double rand = std::rand() % 1000 / 1000. - 0.5;
      ssum += rand * rand;
      pref_direction.push_back(rand);
    }
    ssum = std::sqrt(ssum);

    for (int i = 0; i < dimensions; ++i){
      alpha.push_back((pref_direction.back() / ssum) * scaling);
      pref_direction.pop_back();
    }

    bias = std::rand() % 200; 
  }

        

};
