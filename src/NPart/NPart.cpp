/*
	Copyright (C) 2016-2017 Sotiris Papatheodorou

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

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "NPart.hpp"
#include "NClip.hpp"


/************************************************************/
/********************* Helper functions *********************/
/************************************************************/
n::Polygon halfplane( const n::Point& A, const n::Point& B, const double diam ) {
	/* Initialize halfplane */
	n::Polygon H;
	H.contour.resize(1);
	H.is_hole.resize(1);
	H.is_open.resize(1);
	H.is_hole[0] = false;
	H.is_open[0] = false;
	H.contour[0].resize(4);

	/* Create halfplane assuming both points are on x axis with center on origin */
	H.contour[0][0].x = 0;
	H.contour[0][0].y = -diam;
	H.contour[0][1].x = -diam;
	H.contour[0][1].y = -diam;
	H.contour[0][2].x = -diam;
	H.contour[0][2].y = diam;
	H.contour[0][3].x = 0;
	H.contour[0][3].y = diam;

	/* Rotate halfplane */
	double theta = std::atan2(B.y-A.y, B.x-A.x);
	n::rotate( &H, theta, true);

	/* Translate halfplane */
	n::translate( &H, n::midpoint(A,B) );

	return H;
}

n::Polygon hyperbola_branch( const n::Circle& A, const n::Circle& B, const double diam, const size_t ppb ) {
	/* Initialize hyperbola branch */
	n::Polygon H;


	if (n::is_point(A) && n::is_point(B)) {
		/* If both circles are actually points create the classic voronoi */
		H.contour.resize(1);
		H.is_hole.resize(1);
		H.is_open.resize(1);
		H.is_hole[0] = false;
		H.is_open[0] = false;
		H.contour[0].resize(4);

		H.contour[0][0].x = 0;
		H.contour[0][0].y = -diam;
		H.contour[0][1].x = -diam;
		H.contour[0][1].y = -diam;
		H.contour[0][2].x = -diam;
		H.contour[0][2].y = diam;
		H.contour[0][3].x = 0;
		H.contour[0][3].y = diam;

	} else {
		/* Create hyperbola branch assuming both foci are on x axis with center on origin */
		double a = (A.radius + B.radius) / 2;
		double c = n::dist(A.center, B.center) / 2;
		double b = std::sqrt( c*c - a*a );

		if ( c < a) {
			/* The disks overlap and there is no cell */
			n::make_empty(&H);
			return H;
		} else {
			/* Create all hyperbola branch points using the parametric equation */
			H.contour.resize(1);
			H.is_hole.resize(1);
			H.is_open.resize(1);
			H.is_hole[0] = false;
			H.is_open[0] = false;
			H.contour[0].resize(ppb);

			/* Parameter step */
			double dt = 2*std::acosh(diam/a)/ppb;
			/* Initial parameter value. Parameter range [-pi/2, pi/2] */
			double t = -std::acosh(diam/a) - dt;

			for (size_t i=0; i<ppb; i++) {
				t = t + dt;
				H.contour[0][i].x = -a * std::cosh(t);
				H.contour[0][i].y = b * std::sinh(t);
			}
		}
	}

	/* Rotate halfplane */
	double theta = std::atan2(B.center.y-A.center.y, B.center.x-A.center.x);
	n::rotate( &H, theta, true);

	/* Translate halfplane */
	n::translate( &H, n::midpoint(A.center ,B.center) );

	return H;
}






/**********************************************************/
/********************* Main functions *********************/
/**********************************************************/
void n::info() {
	printf("NPart, version %d.%d\n", N_VERSION_MAJOR, N_VERSION_MINOR);
	#if N_USE_SDL
		printf("NPSDL is available.\n\n");
	#else
		printf("NPSDL is NOT available.\n\n");
	#endif

	printf("Copyright (C) 2016 Sotiris Papatheodorou\n\n");

	printf("This program is free software: you can redistribute it and/or modify\n\
it under the terms of the GNU General Public License as published by\n\
the Free Software Foundation, either version 3 of the License, or\n\
(at your option) any later version.\n\
\n\
This program is distributed in the hope that it will be useful,\n\
but WITHOUT ANY WARRANTY; without even the implied warranty of\n\
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the\n\
GNU General Public License for more details.\n\
\n\
You should have received a copy of the GNU General Public License\n\
along with this program. If not, see http://www.gnu.org/licenses/.\n\n");
}

void n::voronoi( const n::Polygon& region, const n::Points& seeds, n::Polygons* cells) {
	/* Number of seeds */
	size_t N = seeds.size();
	/* Initialize the result */
	cells->resize(N);
	/* Region diameter */
	double diam = n::diameter(region);

	/* Loop over all seed pairs */
	for (size_t i=0; i<N; i++) {
		/* Initialize the cell of i to the region */
		(*cells)[i] = region;
		for (size_t j=0; j<N; j++) {
			if (i != j) {
				/* Create the halfplane containing i with respect to j */
				n::Polygon h = halfplane( seeds[i], seeds[j], diam );
				/* Intersect the current cell with the halfplane with j */
				n::polygon_clip( n::AND, (*cells)[i], h, &((*cells)[i]) );
			}
		}
	}
}

void n::g_voronoi( const n::Polygon& region, const n::Circles& seeds, n::Polygons* cells, const size_t points_per_branch) {
	/* Number of seeds */
	size_t N = seeds.size();
	/* Initialize the result */
	cells->resize(N);
	/* Region diameter */
	double diam = n::diameter(region);

	/* Loop over all seed pairs */
	for (size_t i=0; i<N; i++) {
		/* Initialize the cell of i to the region */
		(*cells)[i] = region;
		for (size_t j=0; j<N; j++) {
			if (i != j) {
				/* Create the hyperbola branch containing i with respect to j */
				n::Polygon h = hyperbola_branch( seeds[i], seeds[j], diam, points_per_branch );
				/* Intersect the current cell with the branch with j */
				n::polygon_clip( n::AND, (*cells)[i], h, &((*cells)[i]) );
			}
		}
	}
}

void n::ys_partitioning( const n::Polygon& region, const n::Polygons& seeds, n::Polygons* cells) {
	/* Number of seeds */
	size_t N = seeds.size();
	/* Initialize the result */
	cells->resize(N+1);

	/* Loop over all seed pairs */
	for (size_t i=0; i<N; i++) {
		/* Initialize the cell of i to the region */
		(*cells)[i] = region;
		for (size_t j=0; j<N; j++) {
			if (i != j) {
				/* Remove the pattern of j from i */
				n::polygon_clip( n::DIFF, (*cells)[i], seeds[j], &((*cells)[i]) );
			} else {
				/* If this is node i just find the intersection of the current cell with the pattern of i */
				/* This is done to constrain the cell of i to the region */
				n::polygon_clip( n::AND, (*cells)[i], seeds[j], &((*cells)[i]) );
			}
		}
	}

	/* Add the common sensed region */
	n::make_empty( &((*cells)[N]) );
	Polygon tmpP;
	for (size_t i=0; i<N; i++) {
		/* Find the common sensed part of i */
		n::polygon_clip( n::DIFF, seeds[i], (*cells)[i], &(tmpP) );
		/* Add the common sensed part ot i to the total */
		n::polygon_clip( n::OR, (*cells)[N], tmpP, &((*cells)[N]) );
	}
	/* Intersect it with the region */
	n::polygon_clip( n::AND, (*cells)[N], region, &((*cells)[N]) );
}

void n::ysuq_partitioning( const n::Polygon& region, const n::Circles& seeds, const std::vector<double>& quality, n::Polygons* cells, bool **neighbors) {
	/* Number of seeds */
	size_t N = seeds.size();
	/* Initialize the result */
	cells->resize(N);
	/* Initialize the neighbors array if required */
	if (neighbors) {
		free(*neighbors);
		*neighbors = (bool*) malloc( N*N*sizeof(bool) );
	}
	/* Convert the circles to polygons */
	n::Polygons sensing;
	sensing.resize(N);
	for (size_t i=0; i<N; i++) {
		sensing = n::Polygons(seeds);
	}

	/* Loop over all nodes */
	for (size_t i=0; i<N; i++) {
		/* Initialize cells to seed polygons */
		(*cells)[i] = sensing[i];

		/* Loop over all other nodes */
		for (size_t j=0; j<N; j++) {
			if (i != j) {
				/* Check for overlapping of the sensing disks */
				if (n::dist(seeds[i].center, seeds[j].center) <= seeds[i].radius + seeds[j].radius) {
					/* Compare coverage quality */
					if (quality[i] < quality[j]) {
						/* Remove the seed polygon of j */
						n::polygon_clip(n::DIFF, (*cells)[i], sensing[j], &((*cells)[i]));
					} else if (quality[i] == quality[j]) {
						/* Use the arbitrary partitioning */
						n::Polygon H = halfplane( seeds[j].center, seeds[i].center, seeds[j].radius );
						n::polygon_clip(n::DIFF, (*cells)[i], H, &((*cells)[i]));
					}

					/* Set neighbor status if required */
					if (neighbors) {
						(*neighbors)[i+N*j] = true;
					}
				} else {
					/* Set neighbor status if required */
					if (neighbors) {
						(*neighbors)[i+N*j] = false;
					}
				}
			} else {
				/* Set neighbor status if required */
				if (neighbors) {
					(*neighbors)[i+N*j] = false;
				}
			}
		}
		/* Intersect the cell with the region */
		n::polygon_clip(n::AND, (*cells)[i], region, &((*cells)[i]));
	}
}
