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

	/* Read disk centers */
	Points P;
	P.push_back( Point(1.256634384155579,0.266874846382719) );
	P.push_back( Point(2.180030882721485,1.17824756490934) );
	P.push_back( Point(2.701234170811698,1.56856761860151) );
	P.push_back( Point(1.232008755529501,0.752191466863367) );

	/* Create disks */
	Circles CC;
	CC.push_back( Circle(P[0], 0.8) );
	CC.push_back( Circle(P[1], 1.0) );
	CC.push_back( Circle(P[2], 0.7) );
	CC.push_back( Circle(P[3], 0.5) );
	Polygons disks;
	disks = Polygons( CC );

	/* Coverage quality */
	std::vector<double> quality;
	quality.push_back(0.8);
	quality.push_back(1.0);
	quality.push_back(0.7);
	quality.push_back(0.5);

	/* Neighbor array */
	bool *neighbors = NULL;

	clock_t begin = clock();
	Polygons YSUQ;
	size_t N = 1;
	for (size_t i=0; i<N; i++) {
		YS_uniform_quality(region, CC, quality, YSUQ, &neighbors);
	}
	clock_t end = clock();
	double YSUQ_time = (double)(end - begin) / CLOCKS_PER_SEC;
	cout << "Created YSUQ " << N << " times." << "\n";
	cout << "Total YSUQ time: " << YSUQ_time << "\n";
	cout << "Average YSUQ time: " << YSUQ_time/N << "\n";


	/* Control law */
	Points V;
	V.resize(P.size());
	for (size_t i=0; i<P.size(); i++) {
		V[i] = YS_uniform_quality_control(region, i, YSUQ, quality, &(neighbors[i]));
		printf("Vx %f  Vy %f  Yz %f\n", V[i].x, V[i].y, V[i].z);
	}





	// YSUQ.print();
	/* Print neighbors */
	for (size_t i=0; i<CC.size(); i++) {
		for (size_t j=0; j<CC.size(); j++) {
			printf("%d ", neighbors[i+CC.size()*j]);
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
			npsdl::plot_points( P );
			npsdl::plot_polygons( disks );

			PLOT_FOREGROUND_COLOR = {0x00, 0xAA, 0x00, 0xFF};
			npsdl::plot_polygons( YSUQ );

			npsdl::plot_render();
			uquit = npsdl::handle_input();
		}
		npsdl::quit_SDL();
	#endif
}
