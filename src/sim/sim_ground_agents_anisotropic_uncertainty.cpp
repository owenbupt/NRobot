/*
	Copyright (C) 2017 Sotiris Papatheodorou

	This file is part of NPart.

    NPart is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    NPart is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NPart.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <cstdio>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>

#include "NR.hpp"


int main() {
	nr::info();

	/****** Simulation parameters ******/
	double Tfinal = 1;
	double Tstep = 0.01;

	/****** Region of interest ******/
	nr::Polygon region;
	nr::read( &region, "resources/region_sq.txt", true);

	/****** Setup agents ******/
    /* Agent initial positions */
	nr::Points P;
	P.push_back( nr::Point(0,0) );
	P.push_back( nr::Point(1.5,0.5) );
	/* Agent initial attitudes */
	nr::Orientations A;
	A.push_back( nr::Orientation(0,0,M_PI/2) );
	A.push_back( nr::Orientation(0,0,M_PI*4/5) );
	/* Number of agents */
	size_t N = P.size();
    /* Sensing, uncertainty and communication radii */
	std::vector<double> sradii { 1.2, 2.1 };
	std::vector<double> uradii { 0.2, 0.2 };
	std::vector<double> cradii (N, 5);
	/* Initialize agents */
	nr::MAs agents (P, A, sradii, uradii, cradii);
	for (size_t i=0; i<N; i++) {
		/* Attitude uncertainty */
		agents[i].attitude_uncertainty = M_PI/10;
		/* Dynamics */
		agents[i].dynamics = nr::DYNAMICS_SI_GROUND_XYy;
		/* Base sensing patterns */
		agents[i].base_sensing = nr::Polygon( nr::Ellipse( 2, 1, nr::Point(1,0) ) );
		/* Sensing quality at relaxed sensing */
		agents[i].relaxed_sensing_quality = 0;
        /* Compute base sensing patterns */
		int err = nr::compute_base_sensing_patterns( &(agents[i]) );
		if (err) {
			std::printf("Clipping operation returned error %d\n", err);
			return nr::ERROR_CLIPPING_FAILED;
		}
	}
	/* Set partitioning and control law */
	nr::set_partitioning( &agents, nr::PARTITIONING_ANISOTROPIC_UNCERTAINTY );
	nr::set_control( &agents, nr::CONTROL_ANISOTROPIC_UNCERTAINTY );

	/****** Initialize plot ******/
	#if NR_PLOT_AVAILABLE
		if (nr::plot_init()) exit(1);
		PLOT_SCALE = 20;
	#endif



	/****** Simulate agents ******/
	size_t smax = std::floor(Tfinal/Tstep);
	bool uquit = false;
	#if NR_TIME_EXECUTION
	clock_t begin, end;
	begin = std::clock();
	#endif

	for (size_t s=1; s<=smax; s++) {
		std::printf("Iteration: %lu\r", s);

		/* Each agent computes its own control input separately */
		for (size_t i=0; i<N; i++) {
            /* Translate and rotate for real sensing */
    		nr::update_sensing_patterns( &(agents[i]) );
			/* Communicate with neighbors and get their states */
			nr::find_neighbors( &(agents[i]), agents );
			/* Compute own cell using neighbors vector */
			nr::compute_cell( &(agents[i]), region );
			/* Compute own control input */
			nr::compute_control( &(agents[i]) );
		}

		nr::print( agents, false );

		/* Plot network state */
		#if NR_PLOT_AVAILABLE
			nr::plot_clear_render();
			nr::plot_show_axes();
			/* Region, nodes and udisks */
			PLOT_FOREGROUND_COLOR = {0x30, 0x30, 0x30, 0xFF};
			nr::plot_polygon( region );
			nr::plot_positions( agents );
			nr::plot_uncertainty( agents );
			/* Red for sdisks */
			PLOT_FOREGROUND_COLOR = {0xAA, 0x00, 0x00, 0xFF};
			nr::plot_sensing( agents );
			/* Blue for cells */
			PLOT_FOREGROUND_COLOR = {0x00, 0x00, 0xAA, 0xFF};
			nr::plot_cells( agents );
			/* Green for communication */
			PLOT_FOREGROUND_COLOR = {0x00, 0xAA, 0x00, 0xFF};
			nr::plot_communication( agents );
			nr::plot_render();

			uquit = nr::plot_handle_input();
			if (uquit) {
				break;
			}
		#endif

		/* The movement of each agent is simulated */
		for (size_t i=0; i<N; i++) {
			nr::simulate_dynamics( &(agents[i]) );
		}

		/************ DEBUG ************/
		// for (size_t i=0; i<N; i++) {
		// 	for (size_t j=0; j<3; j++) {
		// 		std::printf(" %f", agents[i].control_input[j] );
		// 	}
		// 	std::printf("\n");
		// }
		// std::printf("\n");
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
