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
#include <ctime>
#include <NPart.hpp>

#if NP_USE_SDL
	#include <SDL2/SDL.h>
	#include <NPSDL.hpp>
#endif


using namespace std;
using namespace np;



int main() {
	np_info();

	/* Read region */
	Polygon region;
	region.read("Input Files/region_cb.txt");

	/* Read disk centers */
	Points P;
	P.read("Input Files/MED16_final.txt");
	/* Create disks */
	Circles CC;
	for (size_t i=0; i<P.size(); i++) {
		CC.push_back( Circle(P[i], 0.05) );
	}
	/* Create polygons from disks */
	Polygons disks;
	disks = Polygons( CC );

	/* Create GV */
	clock_t begin = clock();
	Polygons GV;
	size_t N = 100;
	for (size_t i=0; i<N; i++) {
		guaranteed_voronoi( region, CC, GV);
	}
	// GV.print();
	clock_t end = clock();
	double GV_time = (double)(end - begin) / CLOCKS_PER_SEC;
	cout << "Created GV " << N << " times." << "\n";
	cout << "Total GV time: " << GV_time << " s" << "\n";
	cout << "Average GV time: " << GV_time/N << " s" << "\n";





	/* Plot */
	#if NP_USE_SDL
		if (npsdl::init_SDL()) exit(1);
		PLOT_SCALE = 150;
		PLOT_X_OFFSET = -1;
		PLOT_Y_OFFSET = -1;
		bool uquit = false;

		while (!uquit) {
			npsdl::clear_render();
			npsdl::show_axes();

			PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
			npsdl::plot_points( P );
			npsdl::plot_polygons( disks );

			PLOT_FOREGROUND_COLOR = {0x00, 0x00, 0xAA, 0xFF};
			npsdl::plot_polygons( GV );

			PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
			npsdl::plot_polygon( region );

			npsdl::plot_render();
			uquit = npsdl::handle_input();
		}
		npsdl::quit_SDL();
	#endif
}
