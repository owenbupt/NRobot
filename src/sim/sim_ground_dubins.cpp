/*
 *  Copyright (C) 2017 Sotiris Papatheodorou
 *
 *  This file is part of NRobot.
 *
 *  NRobot is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  NRobot is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with NRobot.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  Same simulation as Stergiopoulos_Tzes_ICRA14 if uncertainty is set to zero.
 */

#include <cstdio>
#include <cmath>
#include <ctime>
#include <iostream>
#include <chrono>
#include <thread>

#include <NR.hpp>


int main() {
	/****** Simulation parameters ******/
	double Tfinal = 5;
	double Tstep = 0.001;
	bool export_results = true;
	double K = 0.001;

	/* Get the current time. */
	clock_t start_time_raw = std::time(NULL);
	struct tm* start_time = std::localtime( &start_time_raw );
	/* Number of iterations */
	size_t smax = std::floor(Tfinal/Tstep);

	nr::info();

	/****** Region of interest ******/
	nr::Polygon region;
	nr::read( &region, "resources/region_cb.txt", true);
	double rdiameter = nr::diameter(region);

	/****** Setup agents ******/
    /* Agent initial positions */
	nr::Points P;
	P.push_back( nr::Point(1.721,1.013) );
	P.push_back( nr::Point(1.482,1.206) );
	P.push_back( nr::Point(2.006,1.342) );
	P.push_back( nr::Point(1.536,1.454) );
	P.push_back( nr::Point(1.443,1.655) );
	P.push_back( nr::Point(1.792,1.585) );
	P.push_back( nr::Point(1.255,1.134) );
	P.push_back( nr::Point(1.911,0.905) );
	/* Number of agents */
	size_t N = P.size();
	/* Agent initial attitudes */
	nr::Orientations A (N, nr::Orientation(0,0, M_PI/2));
	/* Initialize agents */
	nr::MAs agents ( P, A, Tstep );
	for (size_t i=0; i<N; i++) {
		/* Dynamics */
		// agents[i].dynamics = nr::DYNAMICS_SI_GROUND_XY;
		agents[i].dynamics = nr::DYNAMICS_DUBINS_GROUND_XYy;
		/* Base sensing patterns */
		agents[i].sensing_radius = 0.3;
		agents[i].base_sensing = nr::Polygon( nr::Circle( nr::Point(), agents[i].sensing_radius ) );
		/* Position uncertainty */
		// agents[i].position_uncertainty = 0.1;
		/* Attitude uncertainty */
		// agents[i].attitude_uncertainty = M_PI/10;
		/* Communication radius */
		// agents[i].communication_radius = 2 * agents[i].sensing_radius;
		agents[i].communication_radius = rdiameter;
		/* Increase gain for rotational control law */
		agents[i].control_input_gains[1] = 1;
		// agents[i].control_input_gains[1] = agents[i].time_step;
	}
	/* Set partitioning and control law */
	nr::set_partitioning( &agents, nr::PARTITIONING_VORONOI );
	nr::set_control( &agents, nr::CONTROL_FREE_ARC );


	/****** Initialize MA evolution vector ******/
	std::vector<nr::MA_evolution> agents_evolution (N, nr::MA_evolution());
	for (size_t i=0; i<N; i++) {
		agents_evolution[i] =
		nr::MA_evolution( agents[i].ID, N, smax, agents[i].dynamics );
	}


	/****** Initialize plot ******/
	#if NR_PLOT_AVAILABLE
	if (nr::plot_init()) exit(1);
	PLOT_SCALE = 100;
	bool uquit = false;
	#endif



	/****** Simulate agents ******/
	std::vector<double> H (smax, 0);
	#if NR_TIME_EXECUTION
	clock_t begin, end;
	begin = std::clock();
	#endif

	for (size_t s=1; s<=smax; s++) {

		/* Each agent computes its own control input separately */
		for (size_t i=0; i<N; i++) {
            /* Translate and rotate for real sensing */
    		nr::update_sensing_patterns( &(agents[i]) );
		}
		for (size_t i=0; i<N; i++) {
			/* Communicate with neighbors and get their states */
			nr::find_neighbors( &(agents[i]), agents );
			/* Compute own cell using neighbors vector */
			nr::compute_cell( &(agents[i]), region );
			/* Compute own control input */
			nr::compute_control( &(agents[i]) );
			/* Ensure collision avoidance. */
            nr::ensure_collision_avoidance( &(agents[i]) );
		}

		/* Calculate objective function and print progress. */
		H[s-1] = nr::calculate_objective( agents );
		// std::printf("Iteration: %lu    H: %.4f\r", s, H[s-1]);

		/* Save agent evolution. */
		for (size_t i=0; i<N; i++) {
			agents_evolution[i].position[s-1] = agents[i].position;
			agents_evolution[i].attitude[s-1] = agents[i].attitude;
			agents_evolution[i].velocity_translational[s-1] =
			agents[i].velocity_translational;
			agents_evolution[i].velocity_rotational[s-1] =
			agents[i].velocity_rotational;
			agents_evolution[i].relaxed_sensing_quality[s-1] =
			agents[i].relaxed_sensing_quality;
			for (size_t j=0; j<agents[i].neighbors.size(); j++) {
				agents_evolution[i].
				neighbor_connectivity[agents[i].neighbors[j].ID-1][s-1] = true;
			}
			for (size_t j=0; j<agents_evolution[i].control_input.size(); j++) {
				agents_evolution[i].control_input[j][s-1] =
				agents[i].control_input[j];
			}
		}

		// nr::print( agents, 0 );

		/* Plot network state */
		#if NR_PLOT_AVAILABLE
			nr::plot_clear_render();
			nr::plot_show_axes();

			/* sdisks */
			nr::plot_sensing( agents, RED );
			/* cells */
			nr::plot_cells( agents, BLUE );
			// for (size_t i=0; i<N; i++) {
			// 	nr::plot_cell( agents[i], PLOT_COLORS[i % PLOT_COLORS.size()] );
			// }
			/* communication */
			// nr::plot_communication( agents, GREEN );
			/* Region, nodes and udisks */
			nr::plot_polygon( region, BLACK );
			nr::plot_positions( agents, BLACK );
			nr::plot_uncertainty( agents, BLACK );

			nr::plot_render();
			uquit = nr::plot_handle_input();
			if (uquit) {
				break;
			}
		#endif

		/*
		 *  Add rotational velocity control input.
		 *  APPLIES ONLY TO DUBINS DYNAMICS WITH UNIFORM SENSING.
		 */
		if (s > 1) {
			for (size_t i=0; i<N; i++) {
				if (agents[i].dynamics == nr::DYNAMICS_DUBINS_GROUND_XYy){
					double th_desired_previous =
					  std::atan2(agents_evolution[i].control_input[1][s-2],
					  agents_evolution[i].control_input[0][s-2]);
					double th_desired_current =
					  std::atan2(agents[i].control_input[1],
					  agents[i].control_input[0]);
					double th_current = agents[i].attitude.yaw;
					double th_error_current =
					  th_desired_current - th_current;
					double th_desired_derivative =
					  (th_desired_current - th_desired_previous) /
					  agents[i].time_step;
					agents[i].control_input[2] =
					  K * th_error_current / agents[i].time_step +
					  th_desired_derivative;
				}
			}
		}

		/* The movement of each agent is simulated */
		for (size_t i=0; i<N; i++) {
			nr::simulate_dynamics( &(agents[i]) );
		}
	}



	/****** Print simulation info ******/
	#if NR_TIME_EXECUTION
	end = clock();
	double elapsed_time = (double)(end - begin) / CLOCKS_PER_SEC;
	double average_iteration = elapsed_time / smax;
	std::printf("Simulation finished in %.2f seconds\n", elapsed_time);
	std::printf("Average iteration %.5f seconds\n", average_iteration);
	#endif

	/****** Export simulation results ******/
	if (export_results) {
		int err;
		err = nr::export_simulation_parameters( start_time, N, Tfinal, Tstep,
		elapsed_time, H, region );
		if (err) {
			return nr::ERROR_FILE;
		}
		err = nr::export_agent_parameters( start_time, agents );
		if (err) {
			return nr::ERROR_FILE;
		}
		err = nr::export_agent_state( start_time, agents_evolution );
		if (err) {
			return nr::ERROR_FILE;
		}
	}

	/****** Quit plot ******/
	#if NR_PLOT_AVAILABLE
		uquit = false;
		while (!uquit) {
			nr::plot_clear_render();
			nr::plot_show_axes();

			/* sdisks */
			nr::plot_sensing( agents, RED );
			/* cells */
			nr::plot_cells( agents, BLUE );
			// for (size_t i=0; i<N; i++) {
			// 	nr::plot_cell( agents[i], PLOT_COLORS[i % PLOT_COLORS.size()] );
			// }
			/* communication */
			// nr::plot_communication( agents, GREEN );
			/* Region, nodes and udisks */
			nr::plot_polygon( region, BLACK );
			nr::plot_positions( agents, BLACK );
			nr::plot_uncertainty( agents, BLACK );

			nr::plot_render();
			uquit = nr::plot_handle_input();
			if (uquit) {
				break;
			}
		}
		nr::plot_quit();
	#endif

	return 0;
}
