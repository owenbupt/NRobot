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
#include <NPart.hpp>

#if NP_USE_SDL
	#include <SDL2/SDL.h>
	#include <NPSDL.hpp>
#endif


using namespace std;
using namespace np;



int main() {
	np_info();


	Polygon region;
	region.read("Input Files/region_sq.txt", true);
	// R.rotate(-2.9442);

	Points P;
	P.push_back( Point(-5,5) );
	P.push_back( Point(-5,2) );
	P.push_back( Point(-3,5) );
	P.push_back( Point(-3,-7) );

	Polygons V, GV, YS, disks;
	voronoi( region, P, V);
	cout << endl;
	// V.print();

	Circles CC;
	NPFLOAT r = 1.0;
	CC.push_back( Circle(P[0], 1.8*r) );
	CC.push_back( Circle(P[1], 2.0*r) );
	CC.push_back( Circle(P[2], 1.7*r) );
	CC.push_back( Circle(P[3], 1.5*r) );
	disks = Polygons( CC );

	guaranteed_voronoi( region, CC, GV);
	cout << endl;
	// GV.print();


	// YS_partitioning(region, disks, YS);
	cout << endl;
	// YS.print();

	/* Test point */
	Point TP = Point(-5,5+1.8);
	cout << TP.in(region) << endl;
	cout << TP.in(CC[0]) << endl;





	/* Plot */
	#if NP_USE_SDL
		if (npsdl::init_SDL()) exit(1);
		PLOT_SCALE = 20;
		bool uquit = false;

		while (!uquit) {
			npsdl::clear_render();
			// npsdl::show_axes();

			PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
			npsdl::plot_polygon( region );
			npsdl::plot_points( P );
			npsdl::plot_polygons( disks );
			PLOT_FOREGROUND_COLOR = {0x00, 0xAA, 0x00, 0xFF};
			npsdl::plot_polygons( GV );
			// npsdl::plot_polygons( YS );
			PLOT_FOREGROUND_COLOR = {0x00, 0x00, 0xAA, 0xFF};
			// npsdl::plot_polygons( V );
			// npsdl::plot_polygon( YS[YS.size()-1] );
			PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
			// npsdl::plot_polygon( region );
			npsdl::plot_point( TP );

			npsdl::plot_render();
			uquit = npsdl::handle_input();
		}
		npsdl::quit_SDL();
	#endif
}
