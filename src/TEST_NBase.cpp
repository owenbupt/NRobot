/*
	Copyright (C) 2017 Sotiris Papatheodorou

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

#include "NBase.hpp"
#include <cstdio>
#include <cmath>
#include <iostream>

// using namespace std;

int main() {
	int failed_tests = 0;
	n::Point P, Q, R;

	/****** Point operators ******/
	std::printf("\nTesting Point operators\n");
	/* == */
	P = n::Point();
	Q = n::Point(1, 2, 3);
	if ( (P == P) && (Q == Q) && !(P == Q) ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  Point == Point\n");

	/* != */
	P = n::Point();
	Q = n::Point(1, 2, 3);
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
    P = n::Point();
	Q = n::Point(1, 2, 3);
	if ( (n::norm(P) == 0) && (n::norm(Q) == std::sqrt(14)) ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  norm( Point )\n");

	/* dist() point2point*/
	P = n::Point();
	Q = n::Point(1, 2, 3);
	R = n::Point(1, 0, 3);
	if ( (n::dist(P,Q) == std::sqrt(14)) && (n::dist(Q,R) == 2) ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  dist( Point, Point )\n");

	/* dot() */
	P = n::Point();
	Q = n::Point(1, 2, 3);
	R = n::Point(1, 0, 3);
	if ( (n::dot(P,Q) == 0) && (n::dot(Q,R) == 10) ) {
		std::printf("  [PASS]");
	} else {
		std::printf("  [FAIL]");
		failed_tests++;
	}
	std::printf("  dot( Point, Point )\n");



	if (failed_tests) {
		std::printf("\n%d tests failed\n", failed_tests);
	} else {
		std::printf("\nAll tests passed\n");
	}

    return 0;
}
