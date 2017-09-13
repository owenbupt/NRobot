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
#include <iostream>

#include <NR.hpp>


#define CALC_VORONOI 0
#define CALC_GVORONOI 0
#define CALC_AWGV 1
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

	/* Seed radii */
	std::vector<double> uradii { 0.15, 0.18, 0.1, 0.13 };
	std::vector<double> sradii { 0.8, 1.6, 1.4, 3.5 };
	// std::vector<double> uradii { 0.15, 0.1 };
	// std::vector<double> sradii { 0.8, 1.4 };

	/* Number of seeds */
	size_t N = P.size();

	/* Uncertainty udisks */
	nr::Circles udisks;
	for (size_t i=0; i<N; i++) {
		udisks.push_back( nr::Circle(P[i], uradii[i]) );
	}

	/* Sensing disks */
	nr::Circles sdisks;
	for (size_t i=0; i<N; i++) {
		sdisks.push_back( nr::Circle(P[i], sradii[i]) );
	}
	nr::Polygons poly_sdisks;
	poly_sdisks = nr::Polygons( sdisks );



	/* Voronoi */
	#if CALC_VORONOI
	nr::Polygons V;
	nr::voronoi( region, P, &V);
	#endif

	/* Voronoi cell */
	#if CALC_VORONOI
	nr::Polygons Vc;
	Vc.resize(P.size());
	for (size_t i=0; i<P.size(); i++) {
		nr::voronoi_cell( region, P, i, &(Vc[i]));
	}
	#endif

	/* Guaranteed Voronoi */
	#if CALC_GVORONOI
	nr::Polygons GV;
	nr::g_voronoi( region, udisks, &GV);
	#endif

	/* Guaranteed Voronoi cell */
	#if CALC_GVORONOI
	nr::Polygons GVc;
	GVc.resize(udisks.size());
	for (size_t i=0; i<udisks.size(); i++) {
		nr::g_voronoi_cell( region, udisks, i, &(GVc[i]));
	}
	#endif

	/* AW Guaranteed Voronoi cell */
	#if CALC_AWGV
	nr::Polygons AWGVc;
	AWGVc.resize(udisks.size());
	for (size_t i=0; i<udisks.size(); i++) {
		nr::awg_voronoi_cell( region, udisks, sradii, i, &(AWGVc[i]) );
	}
	// nr::print( AWGVc );
	#endif

	/* YS partitioning */
	#if CALC_YS
	nr::Polygons YS;
	nr::ys_partitioning(region, poly_sdisks, &YS);
	#endif





	/* Plot */
	#if NR_PLOT_AVAILABLE
		if (nr::plot_init()) exit(1);
		PLOT_SCALE = 20;
		bool uquit = false;

		while (!uquit) {
			nr::plot_clear_render();
			nr::plot_show_axes();

			/* White for region and udisks */
			PLOT_FOREGROUND_COLOR = {0x30, 0x30, 0x30, 0xFF};
			nr::plot_polygon( region );
			nr::plot_points( P );
			nr::plot_circles( udisks );

			/* Red for sdisks */
			PLOT_FOREGROUND_COLOR = {0xAA, 0x00, 0x00, 0xFF};
			nr::plot_circles( sdisks );

			/* Green for GV, AWGV and YS */
			PLOT_FOREGROUND_COLOR = {0x00, 0xAA, 0x00, 0xFF};
			#if CALC_GVORONOI
			nr::plot_polygons( GV );
			nr::plot_polygons( GVc );
			for (size_t i=0; i<GVc.size(); i++) {
				// nr::plot_polygon_vertices( GVc[i] );
			}
			#endif
			#if CALC_AWGV
			nr::plot_polygons( AWGVc );
			for (size_t i=0; i<AWGVc.size(); i++) {
				// nr::plot_polygon_vertices( AWGVc[i] );
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

	return 0;
}
