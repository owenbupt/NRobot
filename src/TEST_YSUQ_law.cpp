/*
	Copyright (C) 2016 Sotiris Papatheodorou

	This file is part of NRobot.

    NRobot is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    NRobot is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NRobot.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <cstdio>
#include <iostream>
#include <ctime>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include <NPart/NPart.hpp>
#include <NRobot.hpp>

#if NP_USE_SDL
	#include <SDL2/SDL.h>
	#include <NPart/NPSDL.hpp>
#endif


using namespace std;
using namespace np;
using namespace nr;



int main() {
	np_info();
	nr_info();

	/* Initialize SDL */
	#if NP_USE_SDL
		if (npsdl::init_SDL()) exit(1);
		PLOT_SCALE = 150;
		PLOT_X_OFFSET = -1;
		PLOT_Y_OFFSET = -1;

		npsdl::clear_render();
	#endif

	/* Read region */
	Polygon region;
	region.read("Input Files/region_cb.txt");

	/* Setup robots */
	size_t N = 1;
	std::vector<MAA> robots;
	robots.resize(N);
	Circles sdisks;
	sdisks.resize(N);
	Polygons cells;
	cells.resize(N);
	std::vector<NPFLOAT> quality;
	quality.resize(N);
	Points V;
	V.resize(N);
	bool *neighbors = NULL;

	/* Set robot positions */
	robots[0].position = Point(0.6, 1.1, 2.5-0.01);

	// robots[0].position = Point(1.0, 0.8, 0.6);
	// robots[1].position = Point(1.05, 0.8, 0.7);

	// robots[0].position = Point(1.256634384155579, 0.266874846382719, 0.55);
	// robots[1].position = Point(2.180030882721485, 1.17824756490934, 0.75);
	// robots[2].position = Point(2.701234170811698, 1.56856761860151, 1.55);
	// robots[3].position = Point(1.232008755529501, 0.752191466863367, 2.00);
	for (size_t i=0; i<N; i++) {
		robots[i].zmin = 0.5;
		robots[i].zmax = 2.5;
		robots[i].view_angle = 2.0 * 20.0/180.0 * std::acos(-1.0);
		robots[i].set_quality();
		robots[i].set_sensing();
		robots[i].set_sensing_poly();

		quality[i] = robots[i].quality;
		sdisks[i] = robots[i].sensing;
	}


	size_t smax = 300;
	NPFLOAT dt = 0.1;
	std::chrono::milliseconds plot_sleep(100);

	/* Simulation loop */
	clock_t begin = clock();
	for (size_t s=0; s<smax; s++) {
		printf("---------- %.2f%% ----------\n", 100.0*s/ (double) smax);
		/* Partitioning */
		YS_uniform_quality(region, sdisks, quality, cells, &neighbors);

		/* Copy the cells into the robot class */
		for (size_t i=0; i<N; i++) {
			robots[i].cell = cells[i];
		}


		/* Plot */
		#if NP_USE_SDL
			npsdl::clear_render();
			// npsdl::show_axes();

			npsdl::plot_polygon( region, {0xAA, 0xAA, 0xAA, 0xFF} );
			for (size_t i=0; i<N; i++) {
				npsdl::plot_point( robots[i].position, {0xAA, 0xAA, 0xAA, 0xFF} );
				npsdl::plot_polygon( robots[i].sensing_poly, {0xAA, 0x00, 0x00, 0xFF} );
				npsdl::plot_polygon( robots[i].cell, {0x00, 0xAA, 0x00, 0xFF} );
				npsdl::plot_polygon_vertices( robots[i].cell, {0x00, 0x00, 0x00, 0xFF} );
			}

			npsdl::plot_render();
			if (npsdl::handle_input()) {
				/* User quit */
				exit(0);
			}

			std::this_thread::sleep_for(plot_sleep);
		#endif



		/* Control law */
		for (size_t i=0; i<N; i++) {
			V[i] = YS_uniform_quality_control(region, robots, i, &(neighbors[i]));
			printf("%lu Px %f  Py %f  Pz %f\n", i, robots[i].position.x, robots[i].position.y, robots[i].position.z);
			printf("%lu Vx %f  Vy %f  Vz %f\n", i, V[i].x, V[i].y, V[i].z);
		}


		/* Move robots */
		for (size_t i=0; i<N; i++) {
			robots[i].position += dt * V[i];

			/* Update robot attributes */
			robots[i].set_quality();
			robots[i].set_sensing();
			robots[i].set_sensing_poly();

			quality[i] = robots[i].quality;
			sdisks[i] = robots[i].sensing;
		}
	}
	clock_t end = clock();

	double YSUQ_time = (double)(end - begin) / CLOCKS_PER_SEC;
	cout << "Total simulation time: " << YSUQ_time << "\n";
	cout << "Average iteration time: " << YSUQ_time/smax << "\n";

	free(neighbors);








	/* Plot */
	#if NP_USE_SDL
		char uquit = false;
		while (!uquit) {
			npsdl::clear_render();
			npsdl::show_axes();

			npsdl::plot_polygon( region, {0xAA, 0xAA, 0xAA, 0xFF} );
			for (size_t i=0; i<N; i++) {
				npsdl::plot_point( robots[i].position, {0xAA, 0xAA, 0xAA, 0xFF} );
				npsdl::plot_polygon( robots[i].sensing_poly, {0xAA, 0x00, 0x00, 0xFF} );
				npsdl::plot_polygon( robots[i].cell, {0x00, 0xAA, 0x00, 0xFF} );
				npsdl::plot_polygon_vertices( robots[i].cell, {0x00, 0x00, 0x00, 0xFF} );
			}

			npsdl::plot_render();
			uquit = npsdl::handle_input();
		}
		npsdl::quit_SDL();
	#endif
}
