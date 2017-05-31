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

#include "NRobot.hpp"


int main() {
	nr::info();

	/****** Region of interest ******/
	nr::Polygon region;
	nr::read( &region, "resources/region_cb.txt", true);
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
	std::vector<double> uradii (N, 0);
	std::vector<double> cradii (N, rdiameter);
	/* Sensing disks */
	nr::Circles sensing_disks;
	sensing_disks.push_back( nr::Circle(P[0], sradii[0]) );
	sensing_disks.push_back( nr::Circle(P[1], sradii[1]) );
	sensing_disks.push_back( nr::Circle(P[2], sradii[2]) );
	sensing_disks.push_back( nr::Circle(P[3], sradii[3]) );
	nr::Polygons sensing_polygons;
	sensing_polygons = nr::Polygons( sensing_disks );
	/* Initialize agents */
	nr::MAs agents (P, sradii, uradii, cradii);

	/****** Initialize plot ******/
	#if NR_PLOT_AVAILABLE
		if (nr::plot_init()) exit(1);
		PLOT_SCALE = 20;
	#endif

	/****** Simulate agents ******/
	size_t smax = 1;

	for (size_t s=1; s<=smax; s++) {
		/* Each agent computes its own control input separately */
		for (size_t i=0; i<N; i++) {
			/* Communicate with neighbors and get their states */
				/* Do not copy their neighbor vectors */

			/* Compute own cell using its neighbors vector */

			/* Computes own control input */
		}

		/* Plot network state */
		#if NR_PLOT_AVAILABLE
			nr::plot_clear_render();
			nr::plot_show_axes();
			/* White for region, nodes and udisks */
			PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
			nr::plot_polygon( region );
			nr::plot_points( P );
			/* Red for sdisks */
			PLOT_FOREGROUND_COLOR = {0xAA, 0x00, 0x00, 0xFF};
			nr::plot_circles( sensing_disks );
			/* Blue for cells */
			PLOT_FOREGROUND_COLOR = {0x00, 0x00, 0xAA, 0xFF};
			nr::plot_render();
		#endif

		/* The movement of each agent is simulated */

	}

	/****** Quit plot ******/
	#if NR_PLOT_AVAILABLE
		bool uquit = false;
		while (!uquit) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			uquit = nr::plot_handle_input();
		}
		nr::plot_quit();
	#endif
}
