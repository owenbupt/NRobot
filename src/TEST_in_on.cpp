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
#include <cmath>
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
	region.read("Input Files/region_cb.txt");

	Point P = Point(0,0);
	Circle C = Circle(P, 1.0);
	Polygon CP = Polygon( C, 60 );

	printf("Circle C % f % f  R %f\n", C.center.x, C.center.y, C.radius);

	/* Test point */
	// Point TP = Point(1,0);
	// NPFLOAT t = 1.43001;
	NPFLOAT t = 1.43;
	Point TP = Point(cos(t),sin(t));

	printf("TP % f  % f\n", TP.x, TP.y);
    printf("TP in C %d\n", TP.in(C));
    printf("TP on C %d\n", TP.on(C));
    printf("TP in CP %d\n", TP.in(CP));
    printf("TP on CP %d\n", TP.on(CP));
	// printf("TP dist from CP %.15f\n", TP.dist(CP));
	// printf("TP on_dist CP %d\n", TP.on_dist(CP));
	printf("TP in region %d\n", TP.in(region));
    printf("TP on region %d\n", TP.on(region));
	// printf("TP dist from region %.15f\n", TP.dist(region));
	// printf("TP on_dist region %d\n", TP.on_dist(region));

	size_t i = 0;
	printf("CPi on CP %d\n", CP.contour[0][i].on(CP));
	printf("CPi+ on CP %d\n", CP.contour[0][i+1].on(CP));
	printf("midpt on CP %d\n", midpoint(CP.contour[0][i], CP.contour[0][i+1]).on(CP));
	printf("CPi dist from region %.15f\n", CP.contour[0][i].dist(CP));
	printf("CPi on_dist region %d\n", CP.contour[0][i].on_dist(CP));



	/* Plot */
	#if NP_USE_SDL
		if (npsdl::init_SDL()) exit(1);
		PLOT_SCALE = 150;
		bool uquit = false;

		while (!uquit) {
			npsdl::clear_render();
			npsdl::show_axes();

			npsdl::plot_polygon( region, {0xAA, 0xAA, 0xAA, 0xFF} );
			npsdl::plot_polygon( CP, {0xAA, 0xAA, 0xAA, 0xFF} );
            npsdl::plot_polygon_vertices( CP, {0x00, 0x00, 0x00, 0xFF} );
			npsdl::plot_point( P, {0xAA, 0xAA, 0xAA, 0xFF} );
			npsdl::plot_point( TP, {0x00, 0xAA, 0x00, 0xFF} );

			npsdl::plot_render();
			uquit = npsdl::handle_input();
		}
		npsdl::quit_SDL();
	#endif
}
