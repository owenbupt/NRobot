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

#include "NRPart.hpp"
#if NR_PLOT_AVAILABLE
#include "NRPlot.hpp"
#endif



int main() {
	nr::Polygon region;
	region.read("Input Files/region_sq.txt", true);
	// R.rotate(-2.9442);

	nr::Points P;
	P.push_back( Point(-5,5) );
	P.push_back( Point(-5,2) );
	P.push_back( Point(-3,5) );
	P.push_back( Point(-3,-7) );

	nr::Polygons V, GV, YS, disks;
	nr::voronoi( region, P, V);
	cout << endl;
	// V.print();

	nr::Circles CC;
	double r = 1.0;
	CC.push_back( Circle(P[0], 1.8*r) );
	CC.push_back( Circle(P[1], 2.0*r) );
	CC.push_back( Circle(P[2], 1.7*r) );
	CC.push_back( Circle(P[3], 1.5*r) );
	disks = Polygons( CC );

	nr::guaranteed_voronoi( region, CC, GV);
	cout << endl;
	// GV.print();


	// YS_partitioning(region, disks, YS);
	cout << endl;
	// YS.print();





	/* Plot */
	#if NR_PLOT_AVAILABLE
		if (nr::init_SDL()) exit(1);
		PLOT_SCALE = 20;
		bool uquit = false;

		while (!uquit) {
			nr::clear_render();
			nr::show_axes();

			PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
			nr::plot_polygon( region );
			nr::plot_points( P );
			nr::plot_polygons( disks );
			PLOT_FOREGROUND_COLOR = {0x00, 0xAA, 0x00, 0xFF};
			nr::plot_polygons( GV );
			// nr::plot_polygons( YS );
			PLOT_FOREGROUND_COLOR = {0x00, 0x00, 0xAA, 0xFF};
			// nr::plot_polygons( V );
			// nr::plot_polygon( YS[YS.size()-1] );
			PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
			// nr::plot_polygon( region );
			nr::plot_point( TP );

			nr::plot_render();
			uquit = nr::handle_input();
		}
		nr::quit_SDL();
	#endif
}
