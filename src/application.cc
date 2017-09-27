/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2017 INCF
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


#include "music/application.hh"

#if MUSIC_USE_MPI
#include <mpi.h>
#include <cassert>

namespace MUSIC
{
  static std::string err_not_runtime = "Application not in running state";
  static std::string err_MPI_Init = "MPI_Init was called before the Setup constructor";

  Application::Application ()
    : Application( std::unique_ptr<DefaultContext> (new DefaultContext ()))
  {}

  Application::Application(std::unique_ptr<MusicContext> context, double timebase)
    : Application(context->getConfiguration(), context->getComm(),
		  context->launchedByMusic(), timebase)
  {
    assert (comm_ != MPI::COMM_NULL);
  }

  Application::Application(std::unique_ptr<Configuration> config, MPI::Intracomm comm,
			   bool launchedByMusic, double timebase):
    config_ (std::move (config)),
    timebase_ (timebase),
    comm_ (comm),
    launchedByMusic_ (launchedByMusic),
    port_manager_ (*config_),
    runtime_ (nullptr),
    localTime_ ()
  {
  }

  Application::Application (int& argc, char**& argv, int required, int* provided, double timebase)
    : Application (MusicContextFactory ().createContext (argc, argv, required, provided), timebase)
  {
  }

  Application::Application (int& argc, char**& argv, double timebase)
    : Application (MusicContextFactory ().createContext (argc, argv), timebase)
  {
  }


  Application::~Application()
  {
    if (state_ != ApplicationState::FINALIZED)
      finalize();
  }

  double Application::time() const
  {
    return localTime_.time ();
  }

  void Application::enterSimulationLoop(double h)
  {
    auto localTime_state = localTime_.integerTime ();
    localTime_.configure (timebase_, ClockState (h, timebase_));
    localTime_.set (localTime_state);

    port_manager_.updatePorts ();
    auto ports = port_manager_.getPorts ();
    runtime_.reset (new Runtime (*this, ports, localTime_));
    state_ = ApplicationState::RUNNING;
  }

  void Application::exitSimulationLoop()
  {
    /* MPI::COMM_WORLD.Barrier (); */
    runtime_->finalize ();
    runtime_ = nullptr;
    state_ = ApplicationState::STOPPED;
  }

  // TODO all of this repetetive stuff could be realized as template function
  // but then we have to use a single function name for all of them
  // o


  /* std::shared_ptr<ContInputPort> */
  /* Application::publishContInput (std::string identifier) */
  /* { */
  /* 	return port_manager_.createPort<ContInputPort> (*this, identifier); */
  /* } */


  /* std::shared_ptr<ContOutputPort> */
  /* Application::publishContOutput (std::string identifier) */
  /* { */
  /* 	return port_manager_.createPort<ContOutputPort> (*this, identifier); */
  /* } */


  /* std::shared_ptr<EventInputPort> */
  /* Application::publishEventInput (std::string identifier) */
  /* { */
  /* 	return port_manager_.createPort<EventInputPort> (*this, identifier); */
  /* } */


  /* std::shared_ptr<EventOutputPort> */
  /* Application::publishEventOutput (std::string identifier) */
  /* { */
  /* 	return port_manager_.createPort<EventOutputPort> (*this, identifier); */
  /* } */


  /* std::shared_ptr<MessageInputPort> */
  /* Application::publishMessageInput (std::string identifier) */
  /* { */
  /* 	return port_manager_.createPort<MessageInputPort> (*this, identifier); */
  /* } */


  /* std::shared_ptr<MessageOutputPort> */
  /* Application::publishMessageOutput (std::string identifier) */
  /* { */
  /* 	return port_manager_.createPort<MessageOutputPort> (*this, identifier); */
  /* } */

  PortConnectivityManager& Application::getPortConnectivityManager ()
  {
    return port_manager_;
  }

  double Application::timebase() const
  {
    return timebase_;
  }

  int Application::color () const
  {
    return config_-> Color();
  }

  int Application::nProcs () const
  {
    return comm_.Get_size ();
  }

  int Application::leader () const
  {
    return config_->Leader ();
  }

  std::string Application::name () const
  {
    return config_->Name ();
  }

  const ApplicationMap& Application::applicationMap () const
  {
    return *(config_->applications ());
  }

  MPI::Intracomm Application::communicator() const
  {
    return comm_;
  }

  bool Application::launchedByMusic () const
  {
    return launchedByMusic_;
  }

  void Application::finalize()
  {
    if (state_ == ApplicationState::RUNNING)
      exitSimulationLoop ();
    port_manager_.finalize ();
    state_ = ApplicationState::FINALIZED;
    MPI::Finalize ();
  }

  bool
  Application::config (std::string var, std::string* result)
  {
    return config_->lookup (var, result);
  }


  bool
  Application::config (std::string var, int* result)
  {
    return config_->lookup (var, result);
  }


  bool
  Application::config (std::string var, double* result)
  {
    return config_->lookup (var, result);
  }


}

#endif

