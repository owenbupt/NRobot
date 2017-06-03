/*
	Copyright (C) 2016 Sotiris Papatheodorou

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
#include <iostream>
#include <chrono>
#include <thread>

#include "NR.hpp"


int main() {
	nr::info();

	/****** Simulation parameters ******/
	double Tfinal = 10;
	double Tstep = 0.01;

	/****** Region of interest ******/
	nr::Polygon region;
	nr::read( &region, "resources/region_sq.txt", true);
	double rdiameter = diameter( region );

	/****** Setup agents ******/
	/* Number of agents */
	size_t N = 4;
	/* Node initial positions */
	nr::Points P;
	P.push_back( nr::Point(-5,5) );
	P.push_back( nr::Point(-5,2) );
	P.push_back( nr::Point(-3,5) );
	P.push_back( nr::Point(-3,-7) );
	/* Sensing, uncertainty and communication radii */
	std::vector<double> sradii { 0.8, 1.6, 1.4, 3.5 };
	std::vector<double> uradii { 0.15, 0.18, 0.1, 0.13 };
	std::vector<double> cradii (N, rdiameter);
	/* Initialize agents */
	nr::MAs agents (P, sradii, uradii, cradii);

	/****** Initialize plot ******/
	#if NR_PLOT_AVAILABLE
		if (nr::plot_init()) exit(1);
		PLOT_SCALE = 20;
	#endif

	/****** Simulate agents ******/
	size_t smax = std::floor(Tfinal/Tstep);
	// smax = 1;
	bool uquit = false;
	clock_t begin, end;
	begin = std::clock();

	for (size_t s=1; s<=smax; s++) {
		// std::printf("Iteration: %d\n", s);
		/* Each agent computes its own control input separately */
		for (size_t i=0; i<N; i++) {
			/* Create sensing region */
			nr::create_sensing_disk( &(agents[i]) );

			/* Communicate with neighbors and get their states */
			nr::find_neighbors( &(agents[i]), agents );

			/* Compute own cell using its neighbors vector */
			nr::cell_voronoi( &(agents[i]), region );

			/* Compute own control input */
			nr::control_centroid( &(agents[i]) );
			// nr::control_free_arc( &(agents[i]) );
		}

		// nr::print( agents, false );

		/* Plot network state */
		#if NR_PLOT_AVAILABLE
			nr::plot_clear_render();
			nr::plot_show_axes();
			/* White for region, nodes and udisks */
			PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
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
			agents[i].position += Tstep * agents[i].velocity_translational;
		}
	}

	/****** Print simulation info ******/
	end = clock();
	double elapsed_time = (double)(end - begin) / CLOCKS_PER_SEC;
	double average_iteration = elapsed_time / smax;
	std::printf("Simulation finished in %.2f seconds\n", elapsed_time);
	std::printf("Average iteration %.5f seconds\n", average_iteration);

	/****** Quit plot ******/
	#if NR_PLOT_AVAILABLE
		uquit = false;
		while (!uquit) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			uquit = nr::plot_handle_input();
		}
		nr::plot_quit();
	#endif
}
