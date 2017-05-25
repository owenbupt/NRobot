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


#define CALC_VORONOI 0
#define CALC_GVORONOI 1
#define CALC_YS 0

int main() {
	nr::info();

	/* Region to constrain cells into */
	nr::Polygon region;
	nr::read( &region, "resources/region_sq.txt", true);

	/* Seed points */
	nr::Points P;
	P.push_back( nr::Point(-5,5) );
	P.push_back( nr::Point(-5,2) );
	P.push_back( nr::Point(-3,5) );
	P.push_back( nr::Point(-3,-7) );

	/* Seed disks */
	nr::Circles disks;
	double r = 0.1;
	disks.push_back( nr::Circle(P[0], 1.8*r) );
	disks.push_back( nr::Circle(P[1], 2.0*r) );
	disks.push_back( nr::Circle(P[2], 1.7*r) );
	disks.push_back( nr::Circle(P[3], 1.5*r) );
	nr::Polygons polydisks;
	polydisks = nr::Polygons( disks );

	/* Voronoi */
	nr::Polygons V;
	#if CALC_VORONOI
	nr::voronoi( region, P, &V);
	#endif

	/* Voronoi cell */
	nr::Polygons Vc;
	Vc.resize(P.size());
	for (size_t i=0; i<P.size(); i++) {
		#if CALC_VORONOI
		nr::voronoi_cell( region, P, i, &(Vc[i]));
		#endif
	}

	/* Guaranteed Voronoi */
	nr::Polygons GV;
	#if CALC_GVORONOI
	nr::g_voronoi( region, disks, &GV);
	#endif

	/* Guaranteed Voronoi cell */
	nr::Polygons GVc;
	GVc.resize(disks.size());
	for (size_t i=0; i<disks.size(); i++) {
		#if CALC_GVORONOI
		nr::g_voronoi_cell( region, disks, i, &(GVc[i]));
		#endif
	}

	nr::Polygons YS;
	#if CALC_YS
	nr::ys_partitioning(region, polydisks, &YS);
	#endif





	/* Plot */
	#if NR_PLOT_AVAILABLE
		if (nr::plot_init()) exit(1);
		PLOT_SCALE = 20;
		bool uquit = false;

		while (!uquit) {
			nr::plot_clear_render();
			nr::plot_show_axes();

			/* White for region and seeds */
			PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
			nr::plot_polygon( region );
			nr::plot_points( P );
			nr::plot_polygons( polydisks );

			/* Green for GV and YS */
			PLOT_FOREGROUND_COLOR = {0x00, 0xAA, 0x00, 0xFF};
			#if CALC_GVORONOI
			nr::plot_polygons( GV );
			nr::plot_polygons( GVc );
			for (size_t i=0; i<GVc.size(); i++) {
				nr::plot_polygon_vertices( GVc[i] );
			}
			#endif
			#if CALC_YS
			nr::plot_polygons( YS );
			#endif

			/* Blue for Voronoi and YS common region */
			PLOT_FOREGROUND_COLOR = {0x00, 0x00, 0xAA, 0xFF};
			#if CALC_VORONOI
			nr::plot_polygons( V );
			nr::plot_polygons( Vc );
			#endif
			#if CALC_YS
			nr::plot_polygon( YS[YS.size()-1] );
			#endif

			nr::plot_render();
			uquit = nr::plot_handle_input();
		}
		nr::plot_quit();
	#endif
}
