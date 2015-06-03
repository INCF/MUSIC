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

// VisualiseNeurons.h written by Johannes Hjorth, hjorth@nada.kth.se


#include <GL/gl.h>
#include <GL/freeglut.h>
#include <math.h>
#include <mpi.h>
#include <music.hh>
#include <iostream>
#include <sstream>
#include <vector>
#include "datafile.h"
#include <assert.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/select.h>
#include <getopt.h>
#include <string.h>
#include <vector>
#include <queue>

#ifndef _VISUALISE_NEURONS
#define _VISUALISE_NEURONS

#define DEFAULT_TIMESTEP 1e-3
#define PI 3.141592653589793


  // Inner class used in priority queue
  class TimeIdPair
  {
  public:

    TimeIdPair() { time_ = -1; };
    TimeIdPair(double time, MUSIC::GlobalIndex id) { time_ = time; id_ = id; }

    //bool operator<(const TimeIdPair&) const;
    bool operator<(const TimeIdPair&) const;

    double getTime() const { return time_; }
    MUSIC::GlobalIndex getId() const { return id_; }

  private:
    double time_;
    MUSIC::GlobalIndex id_;
  };



class VisualiseNeurons : public MUSIC::EventHandlerGlobalIndex {

 public:
  VisualiseNeurons() {
    tau_ = 10e-3;      // How fast does activity decay in visualisation?
    time_ = 0;         // Current time
    dt_ = DEFAULT_TIMESTEP;
    stopTime_ = 0;     // End of simulation time
    oldTime_ = 0;      // Time of previous timestep
    done_ = 0;         // Are we there yet?

    maxDist_ = 0;      // Radie of a thought sphere containing all neurons
    is3dFlag_ = 0;     // If it is 3d structure, then lets rotate.

    synchFlag_ = 0;    // Should we try to keep a steady pace or go full speed?
    scaleTime_ = 1;

    gettimeofday(&tickStartTime_,NULL);
    windowTitle_ = "viewevents";
  }
  
  void run(int argc, char **argv);
  void readConfigFile(string filename);
  void finalize();

  // Event handler for incomming spikes
  void operator () ( double t, MUSIC::GlobalIndex id );

  void display();
  void rotateTimer();
  void tick();

  void addNeuron(double x, double y, double z, double r, double cIdx);

  // Static wrapper functions
  static void displayWrapper();
  static void rotateTimerWrapper(int v);
  static void* runMusic(void *arg);

  typedef struct {
    GLdouble x;
    GLdouble y;
    GLdouble z;
    GLdouble r; // radie of neuron
  }neuronCoord;

  typedef struct {
    GLdouble r;
    GLdouble g;
    GLdouble b;
  }neuronColour;



 private:

  void getArgs(int argc, char* argv[]);
  void printHelp();

  MUSIC::Setup* setup_;     // ONLY to be used during setup phase
  MUSIC::Runtime* runtime_; // Music runtime object
  
  GLuint neuronList_;  // OpenGL list for drawing object

  std::vector<neuronCoord> coords_;  // Coordinates of neuron population
  std::vector<double> volt_;  // Activity of neuron population
  std::vector<int> cMap_;   // Which colour does neurons have 

  std::vector<neuronColour> baseLineCol_;  // Colour of resting neuron
  std::vector<neuronColour> excitedCol_;   // Colour of spiking neuron
  double spikeScale_;         // eg, 0.1 = scale up spiking neurons by 10%

  string windowTitle_; 

  double dt_;
  double tau_;       // Tau decay of activity
  double time_;      // Current time
  double oldTime_;   // Previous timestep
  double stopTime_;  // End of simulations
  int done_;         // Simulation done?

  double rotAngle_;  // Current rotation angle

  int rank_;         // Rank of this process

  double maxDist_;   // Distance from origo to outermost neuron
                     // Used when calculating camera position

  int is3dFlag_;

  pthread_t tickThreadID_;

  int synchFlag_;    // Do we try to synch, or run full throttle?
  double scaleTime_; // real time / simulated time
  struct timeval tickStartTime_;
  struct timeval tickEndTime_;
  struct timeval tickDelay_;


  string confFile_;  // Config file with colours and coordinates

  //priority_queue <TimeIdPair> priorityQueue_;
  std::priority_queue<TimeIdPair> priorityQueue_;
};

static std::vector<VisualiseNeurons*> objTable_;



#endif 
