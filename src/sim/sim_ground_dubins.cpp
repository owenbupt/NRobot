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
	size_t plot_sleep_ms = 1;
	bool export_results = false;
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
	/* Uncertainty */
	std::vector<double> position_uncertainty (N,0);
	std::vector<double> attitude_uncertainty (N,0);
	// std::vector<double> position_uncertainty (N,0.1);
	// std::vector<double> attitude_uncertainty (N,M_PI/10);
	std::vector<double> communication_radius (N,rdiameter);
	/* Base sensing */
	std::vector<double> sensing_radius (N, 0.3);
	nr::Polygon base_sensing = nr::Polygon( nr::Circle( nr::Point(), sensing_radius[0] ) );
	/* Control input gains */
	std::vector<double> control_input_gains = {1,1};
	/* Initialize agents */
	nr::MAs agents (
		nr::DYNAMICS_DUBINS_GROUND_XYy,
		nr::PARTITIONING_VORONOI,
		nr::CONTROL_FREE_ARC,
		P,
		A,
		position_uncertainty,
		attitude_uncertainty,
		communication_radius,
		base_sensing,
		control_input_gains,
		Tstep
	);

	/****** Initialize MA evolution vector ******/
	std::vector<nr::MA_evolution> agents_evolution (N, nr::MA_evolution());
	for (size_t i=0; i<N; i++) {
		agents_evolution[i] =
		nr::MA_evolution( agents[i].ID, N, smax, agents[i].dynamics );
	}


	/****** Initialize plot ******/
	#if NR_PLOT_AVAILABLE
	if (nr::plot_init()) exit(1);
	PLOT_SCALE = 200;
	PLOT_X_OFFSET = -300;
	PLOT_Y_OFFSET = 200;
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
			for (size_t i=0; i<N; i++) {
				agents_evolution[i].position.push_back(agents[i].position);
				agents_evolution[i].attitude.push_back(agents[i].attitude);
				agents_evolution[i].velocity_translational.
				  push_back(agents[i].velocity_translational);
				agents_evolution[i].velocity_rotational.
				  push_back(agents[i].velocity_rotational);
				agents_evolution[i].feasible_sensing_quality.
				  push_back(agents[i].feasible_sensing_quality);
				for (size_t j=0; j<agents[i].neighbors.size(); j++) {
					agents_evolution[i].
					  neighbor_connectivity[agents[i].neighbors[j].ID-1].
					  push_back(true);
				}
				for (size_t j=0; j<agents_evolution[i].control_input.size(); j++) {
					agents_evolution[i].control_input[j].push_back
					  (agents[i].control_input[j]);
				}
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
		// nr::plot_communication_links( agents, GREEN );
		/* Region, nodes and udisks */
		nr::plot_polygon( region, BLACK );
		nr::plot_positions( agents, BLACK );
		nr::plot_uncertainty( agents, BLACK );

		nr::plot_render();
		uquit = nr::plot_handle_input();
		if (uquit) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(plot_sleep_ms));
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
