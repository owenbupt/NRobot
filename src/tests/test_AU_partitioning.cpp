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
#include <cstdint>
#include <iostream>

#include <NR.hpp>

int main() {
	nr::info();

	/****** Region of interest ******/
	nr::Polygon region;
	nr::read( &region, "resources/region_sq.txt", true);

	/****** Common Sensing ******/
	nr::Polygon unassigned_region;

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
	nr::MAs agents (P, A, 0.01, sradii, uradii, cradii);
	for (size_t i=0; i<N; i++) {
		/* Attitude uncertainty */
		agents[i].attitude_uncertainty = M_PI/10;
		/* Dynamics */
		agents[i].dynamics = nr::DYNAMICS_SI_GROUND_XYy;
		/* Base sensing patterns */
		agents[i].base_sensing = nr::Polygon( nr::Ellipse( 2, 1, nr::Point(1,0) ) );
		/* Sensing quality at relaxed sensing */
		agents[i].relaxed_sensing_quality = 0;
	}
	/* Set partitioning and control law */
	nr::set_partitioning( &agents, nr::PARTITIONING_ANISOTROPIC_UNCERTAINTY );
	nr::set_control( &agents, nr::CONTROL_FREE_ARC );

	/* Sensing */
	for (size_t i=0; i<N; i++) {
		/* Base sensing patterns */
		int err = nr::compute_base_sensing_patterns( &(agents[i]) );
		if (err) {
			std::printf("Clipping operation returned error %d\n", err);
			return nr::ERROR_CLIPPING_FAILED;
		}
		/* Translate and rotate for real sensing */
		nr::update_sensing_patterns( &(agents[i]) );
	}

	/* Find neighbors and create cells */
	for (size_t i=0; i<N; i++) {
		nr::find_neighbors( &(agents[i]), agents );
		nr::compute_cell( &(agents[i]), region );

		// nr::print( agents[i], 0 );
	}

	/* Find common sensing region */
	nr::make_empty( &unassigned_region );




    /* Plot */
	#if NR_PLOT_AVAILABLE
		if (nr::plot_init()) exit(1);
		PLOT_SCALE = 20;
		bool uquit = false;

		while (!uquit) {
			nr::plot_clear_render();
			nr::plot_show_axes();

			/* Plot region */
			PLOT_FOREGROUND_COLOR = {0x30, 0x30, 0x30, 0xFF};
			nr::plot_polygon( region );

			/* Plot agents */
			PLOT_FOREGROUND_COLOR = {0x30, 0x30, 0x30, 0xFF};
			nr::plot_positions( agents );
			nr::plot_uncertainty( agents );

			/* Plot sensing */
			PLOT_FOREGROUND_COLOR = {0xAA, 0x00, 0x00, 0xFF};
			for (size_t i=0; i<N; i++) {
				// nr::plot_sensing( agents[i] );
			}

			/* Plot grt-sensing */
			PLOT_FOREGROUND_COLOR = {0x00, 0xAA, 0x00, 0xFF};
			for (size_t i=0; i<N; i++) {
				// nr::plot_polygon( agents[i].guaranteed_sensing );
				// nr::plot_polygon( agents[i].relaxed_sensing );
				// nr::plot_polygon( agents[i].total_sensing );
			}

			/* Plot cells */
			PLOT_FOREGROUND_COLOR = {0x00, 0x00, 0xAA, 0xFF};
			for (size_t i=0; i<N; i++) {
				PLOT_FOREGROUND_COLOR = PLOT_COLORS[i];
				nr::plot_cell( agents[i] );
			}

			/* Plot common sensing */
			PLOT_FOREGROUND_COLOR = {0x10, 0x10, 0x10, 0xFF};
			nr::plot_polygon( unassigned_region );

			nr::plot_render();
			uquit = nr::plot_handle_input();
		}
		nr::plot_quit();
	#endif

	return 0;
}
