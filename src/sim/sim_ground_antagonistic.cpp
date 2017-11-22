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

#include <cstdio>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>

#include <NR.hpp>


int main() {
	nr::info();

	/****** Simulation parameters ******/
	double Tfinal = 60;
	double Tstep = 0.01;
	size_t plot_sleep_ms = 0;

	/****** Region of interest ******/
	nr::Polygon region;
	nr::read( &region, "resources/region_cb.txt", true);
	double rdiameter = diameter( region );

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
	/* Sensing, uncertainty and communication radii */
	std::vector<double> sradii (N, 0.5);
	std::vector<double> uradii (N, 0);
	std::vector<double> cradii (N, rdiameter);
	/* Initialize agents */
	nr::MAs agents (P, Tstep, sradii, uradii, cradii);
	/* Set partitioning and control law */
	nr::set_partitioning( &agents, nr::PARTITIONING_VORONOI );
	nr::set_control( &agents, nr::CONTROL_FREE_ARC );
	nr::set_control( &agents, nr::CONTROL_CENTROID );
	// nr::set_control( &agents, nr::CONTROL_R_LIMITED_CENTROID );
	/* Create sensing disks. */
	nr::create_sensing_disks( &agents );
	/* Indices of antagonistic agents. */
	std::vector<bool> antagonist (N, false);
	antagonist[1] = true;
	// antagonist[5] = true;

	/****** Create constrained regions ******/
	nr::Polygons offset_regions;
	for (size_t i=0; i<N; i++) {
		offset_regions.push_back( region );

		int err = nr::offset_in( &(offset_regions[i]), agents[i].position_uncertainty );
		if (err) {
			return nr::ERROR_CLIPPING_FAILED;
		}
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
	size_t smax = std::floor(Tfinal/Tstep);
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
			/* Change control if agent is antagonistic. */
			if (antagonist[i]) {
				for (size_t j=0; j<agents[i].control_input.size(); j++) {
					agents[i].control_input[j] = -agents[i].control_input[j];
				}
			}
			/* Ensure collision avoidance. */
            nr::ensure_collision_avoidance( &(agents[i]) );
		}

		/* Calculate objective function and print progress. */
		H[s-1] = nr::calculate_objective( agents );
		std::printf("Iteration: %lu    H: %.4f\r", s, H[s-1]);

		// nr::print( agents, false );

		/* Plot network state */
		#if NR_PLOT_AVAILABLE
			nr::plot_clear_render();
			nr::plot_show_axes();

			/* Region, nodes and udisks */
			nr::plot_polygon( region, BLACK );
			nr::plot_positions( agents, BLACK );
			nr::plot_uncertainty( agents, BLACK );
			/* sdisks */
			nr::plot_sensing( agents, RED );
			/* cells */
			nr::plot_cells( agents, BLUE );
			/* communication */
			// nr::plot_communication( agents, GREEN );
			/* Mark antagonistic agents. */
			for (size_t i=0; i<N; i++) {
				if (antagonist[i]) {
					plot_point( agents[i].position, RED, 3 );
					plot_cell( agents[i], RED );
				}
			}

			nr::plot_render();
			uquit = nr::plot_handle_input();
			if (uquit) {
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(plot_sleep_ms));
		#endif

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

	/****** Quit plot ******/
	#if NR_PLOT_AVAILABLE
		uquit = false;
		while (!uquit) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			uquit = nr::plot_handle_input();
		}
		nr::plot_quit();
	#endif

	return 0;
}
