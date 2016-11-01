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

	/* Read region */
	Polygon region;
	region.read("Input Files/region_cb.txt");

	/* Setup robots */
	size_t N = 4;
	std::vector<MAA> robots;
	robots.resize(N);

	Circles sdisks;
	sdisks.resize(N);
	Polygons cells;
	cells.resize(N);
	std::vector<NPFLOAT> quality;
	quality.resize(N);

	/* Set robot positions */
	robots[0].position = Point(1.256634384155579, 0.266874846382719, 0.55);
	robots[1].position = Point(2.180030882721485, 1.17824756490934, 0.75);
	robots[2].position = Point(2.701234170811698, 1.56856761860151, 1.55);
	robots[3].position = Point(1.232008755529501, 0.752191466863367, 2.00);
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

	/* Neighbor array */
	bool *neighbors = NULL;

	clock_t begin = clock();
	Polygons YSUQ;
	size_t M = 1;
	for (size_t i=0; i<M; i++) {
		YS_uniform_quality(region, sdisks, quality, cells, &neighbors);
	}
	clock_t end = clock();
	double YSUQ_time = (double)(end - begin) / CLOCKS_PER_SEC;
	cout << "Created YSUQ " << M << " times." << "\n";
	cout << "Total YSUQ time: " << YSUQ_time << "\n";
	cout << "Average YSUQ time: " << YSUQ_time/M << "\n";

	/* Copy the cells into the robot class */
	for (size_t i=0; i<N; i++) {
		robots[i].cell = cells[i];
	}


	/* Control law */
	Points V;
	V.resize(N);
	for (size_t i=0; i<N; i++) {
		V[i] = YS_uniform_quality_control(region, robots, i, &(neighbors[i]));
		printf("Vx %f  Vy %f  Yz %f\n", V[i].x, V[i].y, V[i].z);
	}





	// cells.print();
	/* Print neighbors */
	for (size_t i=0; i<N; i++) {
		for (size_t j=0; j<N; j++) {
			printf("%d ", neighbors[i+N*j]);
		}
		printf("\n");
	}
	free(neighbors);








	/* Plot */
	#if NP_USE_SDL
		if (npsdl::init_SDL()) exit(1);
		PLOT_SCALE = 150;
		PLOT_X_OFFSET = -1;
		PLOT_Y_OFFSET = -1;
		char uquit = false;

		while (!uquit) {
			npsdl::clear_render();
			// npsdl::show_axes();

			PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
			npsdl::plot_polygon( region );
			for (size_t i=0; i<N; i++) {
				npsdl::plot_point( robots[i].position );
				npsdl::plot_polygon( robots[i].sensing_poly );
			}

			PLOT_FOREGROUND_COLOR = {0x00, 0xAA, 0x00, 0xFF};
			npsdl::plot_polygons( cells );

			npsdl::plot_render();
			uquit = npsdl::handle_input();
		}
		npsdl::quit_SDL();
	#endif
}
