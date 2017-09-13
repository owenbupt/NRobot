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
#include <cmath>
#include <iostream>

#include <NR.hpp>

int main() {
	nr::info();

	int failed_tests = 0;
	nr::Point P, Q, R;

	/****** Point operators ******/
	std::printf("\nTesting Point operators\n");
	/* == */
	P = nr::Point();
	Q = nr::Point(1, 2, 3);
	if ( (P == P) && (Q == Q) && !(P == Q) ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  Point == Point\n");

	/* != */
	P = nr::Point();
	Q = nr::Point(1, 2, 3);
	if ( !(P != P) && !(Q != Q) && (P != Q) ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  Point != Point\n");

	/****** Point functions ******/
    std::printf("\nTesting Point functions\n");
	/* norm() */
    P = nr::Point();
	Q = nr::Point(1, 2, 3);
	if ( (nr::norm(P) == 0) && (nr::norm(Q) == std::sqrt(14)) ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  norm( Point )\n");

	/* dist() point2point*/
	P = nr::Point();
	Q = nr::Point(1, 2, 3);
	R = nr::Point(1, 0, 3);
	if ( (nr::dist(P,Q) == std::sqrt(14)) && (nr::dist(Q,R) == 2) ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  dist( Point, Point )\n");

	/* dot() */
	P = nr::Point();
	Q = nr::Point(1, 2, 3);
	R = nr::Point(1, 0, 3);
	if ( (nr::dot(P,Q) == 0) && (nr::dot(Q,R) == 10) ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  dot( Point, Point )\n");

	/* Testing other functions */
	std::printf("\nTesting other functions\n");
	P = nr::Point(1, 1);
	Q = nr::Point(std::sqrt(2), M_PI/4);
	R = nr::cart2pol(P);
	if ( R == Q ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  cart2pol( Point )\n");
	if ( nr::pol2cart(R) == P ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  pol2cart( Point )\n");


	/****** Point in/on functions ******/
	nr::Polygon polygon;
	nr::read( &polygon, "resources/region_cb.txt", true);

	P = nr::Point(2.9325, 1.7);
	if ( nr::in( P, polygon ) ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  in( Point, Polygon )\n");
	if ( nr::on( P, polygon ) ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  on( Point, Polygon )\n");
	if ( nr::is_vertex_of( P, polygon ) ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  is_vertex_of( Point, Polygon )\n");


	/* Plot */
	#if NR_PLOT_AVAILABLE
		if (nr::plot_init()) exit(1);
		PLOT_SCALE = 20;
		bool uquit = false;
		while (!uquit) {
			nr::plot_clear_render();
			nr::plot_show_axes();
			PLOT_FOREGROUND_COLOR = {0x30, 0x30, 0x30, 0xFF};

			/* Plot polygon */
			nr::plot_polygon( polygon );
			/* Plot point */
			nr::plot_point( P );

			nr::plot_render();
			uquit = nr::plot_handle_input();
		}
		nr::plot_quit();
	#endif



	/* Finished testing */
	if (failed_tests) {
		std::printf("\n%d tests failed\n", failed_tests);
	} else {
		std::printf("\nAll tests passed\n");
	}

    return 0;
}
