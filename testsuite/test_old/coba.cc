/*
 *  This file is part of MUSIC.
 *  Copyright (C) 2007 CSC, KTH
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

#define N 4000

#define T 5.0
#define DT 1e-4
#define PRINT_EVERY_NTH 100
#define N_PLOTTED_ECELLS 5
#define N_PLOTTED_ICELLS 5

#define cell_area 20000e-12
#define tau_m        20e-3
#define C_m           1e-2
#define rho_leak      5e-1
#define Erev_leak   -65e-3

#define Gexc	      6e-9
#define Ginh	     67e-9
#define Erev_exc      0
#define Erev_inh    -80e-3
#define tau_exc       5e-3
#define tau_inh      10e-3

#define conn_prob     0.02
#define conn_delay    1e-6 // arbitrary non-zero

#define gNa	     0.1e4
#define gKd	    0.03e4
#define ENa	     50e-3
#define EK	    -90e-3
#define Vtr	    -63e-3

#define N_e ((int) (0.8 * N))
#define N_i (N - N_e)

#include <iostream>
#include <cmath>

#include <split.hh>
#include <split/connection-set.hh>

using namespace libsplit;

class projection;

class cells
{
private:
  split_int* split;
  int _pop_id;
  int _leak_id;
  int _na_id;
  int _k_id;
  int _size;

public:
  cells (split_int* _split, int size);
  int pop_id () { return _pop_id; }
  int na_id () { return _na_id; }
  int size () { return _size; }
  
  projection* project (cells* post);
  
  void set_parameters ();
};

class projection
{
  split_int* split;
  int proj_id;
  connection_set c;
public:
  projection (split_int* _split, cells& pre, cells& post)
    : split (_split), c (random_uniform (conn_prob))
  {
    c.init (0, pre.size (), 0, post.size ());
    proj_id = split->create_projection (CHEMICAL_SYNAPSE, SCALAR,
					pre.pop_id (), CONTRIB, pre.na_id (),
					post.pop_id (), 0,
					c.size ());
    for (c.begin (); !c.end (); ++c)
      {
	split->set_projection_index_int (proj_id, SYN_PRE, c.id (), c.pre ());
	split->set_projection_index_int (proj_id, SYN_POST, c.id (), c.post ());
	split->set_projection_index_real (proj_id, SYN_DELAY, c.id (),
					  conn_delay);
      }
  }
  
  void set_parameters (double G, double Erev, double tau)
  {
    split->set_projection_scalar_real (proj_id, SYN_E, Erev);
    split->set_projection_scalar_real (proj_id, SYN_DURATION, 0.0);
    split->set_projection_scalar_real (proj_id, SYN_RAISE, 0.0);
    split->set_projection_scalar_real (proj_id, SYN_DECAY, tau);
    for (c.begin (); !c.end (); ++c)
      split->set_projection_index_real (proj_id, SYN_G, c.id (), G);
  }
};

cells::cells (split_int* _split, int size)
  : split (_split), _size (size)
{
  int tree_map[1] = { 0 };
  _pop_id = split->create_population ();
  split->create_electric_tree (_pop_id, ELECTRIC_SCALAR_STAR, N_e, 0, 0,
			       tree_map /* dummy */ );
  _leak_id = split->create_e_leak (_pop_id, N_e, 0);
  _na_id = split->create_simple_channel (_pop_id, TRAUB_NA, SCALAR, N_e, 0);
  _k_id = split->create_simple_channel (_pop_id, HH_K, SCALAR, N_e, 0);
}

projection*
cells::project (cells* post)
{
  return new projection (split, *this, *post);
}
  
void
cells::set_parameters ()
{
  split->set_scalar_real (_pop_id, ELECTRIC, 0, CM, cell_area * C_m);
  split->set_scalar_real (_pop_id, ELECTRIC, 0, GCORE, 0);
  split->set_scalar_real (_pop_id, CONTRIB, _leak_id, E_LEAK_E, Erev_leak);
  for (int i = 0; i < _size; ++i)
    {
      split->set_index_real (_pop_id, ELECTRIC, 0, INITIAL, i, Erev_leak);
      split->set_index_real (_pop_id, CONTRIB, _leak_id,
			     E_LEAK_G_M, i, cell_area * rho_leak);
    }

  // Na activation
  split->set_scalar_real (_pop_id, CONTRIB, _na_id,
			  ACT_ALPHA_A, 0.32e6);
  split->set_scalar_real (_pop_id, CONTRIB, _na_id,
			  ACT_ALPHA_B, 13e-3 + Vtr);
  split->set_scalar_real (_pop_id, CONTRIB, _na_id,
			  ACT_ALPHA_C, 4e-3);
  split->set_scalar_real (_pop_id, CONTRIB, _na_id,
			  ACT_BETA_A, 0.28e6);
  split->set_scalar_real (_pop_id, CONTRIB, _na_id,
			  ACT_BETA_B, 40e-3 + Vtr);
  split->set_scalar_real (_pop_id, CONTRIB, _na_id,
			  ACT_BETA_C, 5e-3);
  split->set_scalar_int (_pop_id, CONTRIB, _na_id,
			 ACT_EXPO, 3);
  // Na inactivation
  split->set_scalar_real (_pop_id, CONTRIB, _na_id,
			  INACT_ALPHA_A, 0.128e3);
  split->set_scalar_real (_pop_id, CONTRIB, _na_id,
			  INACT_ALPHA_B, 17e-3 + Vtr);
  split->set_scalar_real (_pop_id, CONTRIB, _na_id,
			  INACT_ALPHA_C, 18e-3);
  split->set_scalar_real (_pop_id, CONTRIB, _na_id,
			  INACT_BETA_A, 4e3);
  split->set_scalar_real (_pop_id, CONTRIB, _na_id,
			  INACT_BETA_B, 40e-3 + Vtr);
  split->set_scalar_real (_pop_id, CONTRIB, _na_id,
			  INACT_BETA_C, 5e-3);
  split->set_scalar_int (_pop_id, CONTRIB, _na_id,
			 INACT_EXPO, 1);
  // K
  split->set_scalar_real (_pop_id, CONTRIB, _k_id,
			  ACT_ALPHA_A, 0.032e6);
  split->set_scalar_real (_pop_id, CONTRIB, _k_id,
			  ACT_ALPHA_B, 15e-3 + Vtr);
  split->set_scalar_real (_pop_id, CONTRIB, _k_id,
			  ACT_ALPHA_C, 5e-3);
  split->set_scalar_real (_pop_id, CONTRIB, _k_id,
			  ACT_BETA_A, 0.5e3);
  split->set_scalar_real (_pop_id, CONTRIB, _k_id,
			  ACT_BETA_B, 10e-3 + Vtr);
  split->set_scalar_real (_pop_id, CONTRIB, _k_id,
			  ACT_BETA_C, 40e-3);
  split->set_scalar_int (_pop_id, CONTRIB, _k_id,
			 ACT_EXPO, 4);

  for (int i = 0; i < _size; ++i)
    {
      // Na
      split->set_index_real (_pop_id, CONTRIB, _na_id,
			     CHAN_E, i, ENa);
      split->set_index_real (_pop_id, CONTRIB, _na_id,
			     CHAN_G, i, cell_area * gNa);
      // K
      split->set_index_real (_pop_id, CONTRIB, _k_id,
			     CHAN_E, i, EK);
      split->set_index_real (_pop_id, CONTRIB, _k_id,
			     CHAN_G, i, cell_area * gKd);
    }
}

class noise {
  split_int* split;
  cells* c;
  int id;
public:
  noise (split_int*  _split, cells* _c)
    : split (_split), c (_c)
  {
    id = split->create_noise (c->pop_id (), c->size (), 0);
  }
  void set_parameters ()
  {
    for (int i = 0; i < c->size (); ++i)
      {
	split->set_index_real (c->pop_id (), CONTRIB, id,
			       NOISE_E, i, 0);
	split->set_index_real (c->pop_id (), CONTRIB, id,
			       NOISE_G, i, 7.5e-10);
	split->set_index_real (c->pop_id (), CONTRIB, id,
			       NOISE_INTENSITY, i, 100);
	split->set_index_real (c->pop_id (), CONTRIB, id,
			       NOISE_INITIAL, i, 0);
	split->set_index_real (c->pop_id (), CONTRIB, id,
			       NOISE_TAU, i, 0.01);
	split->set_index_real (c->pop_id (), CONTRIB, id,
			       NOISE_START_TIME, i, 0);
	split->set_index_real (c->pop_id (), CONTRIB, id,
			       NOISE_END_TIME, i, 0.05);
      }
  }
};

void
setup_volts_plot (split_int* split, int id, cells* c, int n, int& idx)
{
  for (int i = 0; i < n; ++i)
    {
      split->set_plot_index_int (id, idx, PLOT_ID, 0);
      split->set_plot_index_int (id, idx, PLOT_POP_ID, c->pop_id ());
      split->set_plot_index_int (id, idx, PLOT_TYPE, ELECTRIC);
      split->set_plot_index_int (id, idx, PLOT_TAG, STATE_SV);
      split->set_plot_index_int (id, idx, PLOT_IDX, i);
      split->set_plot_index_real (id, idx, PLOT_MIN, -0.08);
      split->set_plot_index_real (id, idx, PLOT_MAX, 0.05);
      ++idx;
    }
}

int
main (int argc, char **argv)
{
  std::cout << "Setup\n";
  
  split_int* split = new split_int (&argc, &argv);
  
  // Create cells
  cells* ecells = new cells (split, N_e);
  cells* icells = new cells (split, N_i);
  
  // Create synapses
  projection* ee = ecells->project (ecells);
  projection* ei = ecells->project (icells);
  projection* ii = icells->project (icells);
  projection* ie = icells->project (ecells);

  // Create ports
  int n_ports = 0;
  while (true)
    {
      std::ostrstream portname;
      portname << "ein" << n_ports;
      if (is_port (portname.str ()))
	{
	  split->create_spike_input (portname.str (), ecells->pop_id (), 0, N);
	  std::ostrstream portname;
	  portname << "iin" << n_ports;
	  if (is_port (portname.sr ()))
	    split->create_spike_input (portname.str (), icells->pop_id (), 0, N);
	  ++n_ports;
	}
      else
	break;
    }
  if (n_ports > 0)
    {
      split->create_spike_output ("eout", ecells->pop_id (), CONTRIB, ecells->na_id ());
      split->create_spike_output ("iout", icells->pop_id (), CONTRIB, icells->na_id ());
    }
  
  // Noiseinjection
  noise* enoise = new noise (split, ecells);
  noise* inoise = new noise (split, icells);
  
  // Volt plots
  int volts_id = split->create_plot ("volts.out",
				     N_PLOTTED_ECELLS + N_PLOTTED_ICELLS);
  split->set_plot_scalar_int (volts_id, PLOT_EVERY_NTH, 1);
  int idx = 0;
  setup_volts_plot (split, volts_id, ecells, N_PLOTTED_ECELLS, idx);
  setup_volts_plot (split, volts_id, icells, N_PLOTTED_ICELLS, idx);
  
  // Spike plot
  int spikes_id = split->create_spike_plot_ranges ("spikes.out");
  split->set_plot_range (spikes_id, ecells->pop_id (), ELECTRIC, 0,
			 STATE_SV, 0, ecells->size ());
  split->set_plot_range (spikes_id, icells->pop_id (), ELECTRIC, 0,
			 STATE_SV, 0, icells->size ());
  
  // Distribute simulation onto slaves
  split->map (0);
  
  // Cell parameters
  ecells->set_parameters ();
  icells->set_parameters ();
  
  // Synaptic parameters
  ee->set_parameters (Gexc, Erev_exc, tau_exc);
  ei->set_parameters (Gexc, Erev_exc, tau_exc);
  ii->set_parameters (Ginh, Erev_inh, tau_inh);
  ie->set_parameters (Ginh, Erev_inh, tau_inh);

  // Noise parameters
  enoise->set_parameters ();
  inoise->set_parameters ();
  
  // Allocate
  split->allocate (0.0, T, DT);
  
  // Initialize
  split->initialize (DT);
  
  // Simulate
  std::cout << "Simulate\n";
  int total_steps = (int) ceil (T / DT);
  for (int steps = 0; steps < total_steps; steps += PRINT_EVERY_NTH)
    split->simulate (PRINT_EVERY_NTH);
  
  // Quit
  split->quit ();
  
  return 0;
}
