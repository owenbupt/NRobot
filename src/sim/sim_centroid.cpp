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

#include "NRobot.hpp"


int main() {
	nr::info();

	/****** Region of interest ******/
	nr::Polygon region;
	nr::read( &region, "resources/region_cb.txt", true);

	/****** Setup agents ******/
	/* Number of agents */
	size_t N = 4;
	/* Node initial positions */
	nr::Points P;
	P.push_back( nr::Point(-5,5) );
	P.push_back( nr::Point(-5,2) );
	P.push_back( nr::Point(-3,5) );
	P.push_back( nr::Point(-3,-7) );
	/* Sensing disks */
	double r = 0.8;
	std::vector<double> sradii;
	sradii.push_back( 1.0*r );
	sradii.push_back( 2.0*r );
	sradii.push_back( 1.7*r );
	sradii.push_back( 4.5*r );
	nr::Circles sdisks;
	sdisks.push_back( nr::Circle(P[0], sradii[0]) );
	sdisks.push_back( nr::Circle(P[1], sradii[1]) );
	sdisks.push_back( nr::Circle(P[2], sradii[2]) );
	sdisks.push_back( nr::Circle(P[3], sradii[3]) );
	nr::Polygons poly_sdisks;
	poly_sdisks = nr::Polygons( sdisks );
	/* Initialize agents */
	std::vector<nr::MA> agents[N];
	for (size_t i=0; i<N; i++) {
		agents[i] = nr::MA( P[i], sradii[i] );
		agents[i].sensing = poly_sdisks[i];
	}



	/****** Simulate agents ******/
	size_t smax = 100;

	for (size_t s=1; s<=smax; s++) {
		/* Communicate with neighbors and get their states */
			/* Do not copy their neighbor vectors */

		/* Each agent computes its cell using its neighbors vector */
		for (size_t i=0; i<N; i++) {
			// nr::voronoi_cell( region, agents[i].position, i, &(agents[i].cell));
		}

		/* Plot network state */
		#if NR_PLOT_AVAILABLE
			if (nr::plot_init()) exit(1);
			PLOT_SCALE = 20;
			bool uquit = false;

			while (!uquit) {
				nr::plot_clear_render();
				nr::plot_show_axes();

				/* White for region and udisks */
				PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
				nr::plot_polygon( region );
				nr::plot_points( P );

				/* Red for sdisks */
				PLOT_FOREGROUND_COLOR = {0xAA, 0x00, 0x00, 0xFF};
				nr::plot_circles( sdisks );

				/* Blue for Voronoi and YS common region */
				PLOT_FOREGROUND_COLOR = {0x00, 0x00, 0xAA, 0xFF};
				nr::plot_polygons( Vc );

				nr::plot_render();
				uquit = nr::plot_handle_input();
			}
			nr::plot_quit();
		#endif


		/* Each agent computes its control input */

		/* The movement of each agent is simulated */
		
	}
}
