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

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "NPart.hpp"
#include "NPClip.hpp"


/************************************************************/
/********************* Helper functions *********************/
/************************************************************/
np::Polygon halfplane( const np::Point& A, const np::Point& B, const NPFLOAT diam ) {
	/* Initialize halfplane */
	np::Polygon H;
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
	NPFLOAT theta = std::atan2(B.y-A.y, B.x-A.x);
	H.rotate(theta, true);

	/* Translate halfplane */
	H.translate( midpoint(A,B) );

	return H;
}

np::Polygon hyperbola_branch( const np::Circle& A, const np::Circle& B, const NPFLOAT diam, const size_t ppb ) {
	/* Initialize hyperbola branch */
	np::Polygon H;


	if (A.is_point() && B.is_point()) {
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
		NPFLOAT a = (A.radius + B.radius) / 2;
		NPFLOAT c = dist(A.center, B.center) / 2;
		NPFLOAT b = std::sqrt( c*c - a*a );

		if ( c < a) {
			/* The disks overlap and there is no cell */
			H.make_empty();
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
			NPFLOAT dt = 2*std::acosh(diam/a)/ppb;
			/* Initial parameter value. Parameter range [-pi/2, pi/2] */
			NPFLOAT t = -std::acosh(diam/a) - dt;

			for (size_t i=0; i<ppb; i++) {
				t = t + dt;
				H.contour[0][i].x = -a * std::cosh(t);
				H.contour[0][i].y = b * std::sinh(t);
			}
		}
	}

	/* Rotate halfplane */
	NPFLOAT theta = std::atan2(B.center.y-A.center.y, B.center.x-A.center.x);
	H.rotate(theta, true);

	/* Translate halfplane */
	H.translate( midpoint(A.center ,B.center) );

	return H;
}






/**********************************************************/
/********************* Main functions *********************/
/**********************************************************/
void np::np_info() {
	printf("NPart, version %d.%d\n", NP_VERSION_MAJOR, NP_VERSION_MINOR);
	#if NP_USE_SDL
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

void np::voronoi( const np::Polygon& region, const np::Points& seeds, np::Polygons& cells) {
	/* Number of seeds */
	size_t N = seeds.size();
	/* Initialize the result */
	cells.resize(N);
	/* Region diameter */
	NPFLOAT diam = region.diameter();

	/* Loop over all seed pairs */
	for (size_t i=0; i<N; i++) {
		/* Initialize the cell of i to the region */
		cells[i] = region;
		for (size_t j=0; j<N; j++) {
			if (i != j) {
				/* Create the halfplane containing i with respect to j */
				np::Polygon h = halfplane( seeds[i], seeds[j], diam );
				/* Intersect the current cell with the halfplane with j */
				npclip::polygon_clip( npclip::AND, cells[i], h, cells[i] );
			}
		}
	}
}

void np::guaranteed_voronoi( const np::Polygon& region, const np::Circles& seeds, np::Polygons& cells, const size_t points_per_branch) {
	/* Number of seeds */
	size_t N = seeds.size();
	/* Initialize the result */
	cells.resize(N);
	/* Region diameter */
	NPFLOAT diam = region.diameter();

	/* Loop over all seed pairs */
	for (size_t i=0; i<N; i++) {
		/* Initialize the cell of i to the region */
		cells[i] = region;
		for (size_t j=0; j<N; j++) {
			if (i != j) {
				/* Create the hyperbola branch containing i with respect to j */
				np::Polygon h = hyperbola_branch( seeds[i], seeds[j], diam, points_per_branch );
				/* Intersect the current cell with the branch with j */
				npclip::polygon_clip( npclip::AND, cells[i], h, cells[i] );
			}
		}
	}
}

void np::YS_partitioning( const np::Polygon& region, const np::Polygons& seeds, np::Polygons& cells) {
	/* Number of seeds */
	size_t N = seeds.size();
	/* Initialize the result */
	cells.resize(N+1);

	/* Loop over all seed pairs */
	for (size_t i=0; i<N; i++) {
		/* Initialize the cell of i to the region */
		cells[i] = region;
		for (size_t j=0; j<N; j++) {
			if (i != j) {
				/* Remove the pattern of j from i */
				npclip::polygon_clip( npclip::DIFF, cells[i], seeds[j], cells[i] );
			} else {
				/* If this is node i just find the intersection of the current cell with the pattern of i */
				/* This is done to constrain the cell of i to the region */
				npclip::polygon_clip( npclip::AND, cells[i], seeds[j], cells[i] );
			}
		}
	}

	/* Add the common sensed region */
	cells[N].make_empty();
	Polygon tmpP;
	for (size_t i=0; i<N; i++) {
		/* Find the common sensed part of i */
		npclip::polygon_clip( npclip::DIFF, seeds[i], cells[i], tmpP );
		/* Add the common sensed part ot i to the total */
		npclip::polygon_clip( npclip::OR, cells[N], tmpP, cells[N] );
	}
	/* Intersect it with the region */
	npclip::polygon_clip( npclip::AND, cells[N], region, cells[N] );
}

void np::YS_uniform_quality( const np::Polygon& region, const np::Circles& seeds, const std::vector<double>& quality, np::Polygons& cells, bool **neighbors) {
	/* Number of seeds */
	size_t N = seeds.size();
	/* Initialize the result */
	cells.resize(N);
	/* Initialize the neighbors array if required */
	if (neighbors) {
		free(*neighbors);
		*neighbors = (bool*) malloc( N*N*sizeof(bool) );
	}
	/* Convert the circles to polygons */
	np::Polygons sensing;
	sensing.resize(N);
	for (size_t i=0; i<N; i++) {
		sensing = Polygons(seeds);
	}

	/* Loop over all nodes */
	for (size_t i=0; i<N; i++) {
		/* Initialize cells to seed polygons */
		cells[i] = sensing[i];

		/* Loop over all other nodes */
		for (size_t j=0; j<N; j++) {
			if (i != j) {
				/* Check for overlapping of the sensing disks */
				if (np::dist(seeds[i].center, seeds[j].center) <= seeds[i].radius + seeds[j].radius) {
					/* Compare coverage quality */
					if (quality[i] < quality[j]) {
						/* Remove the seed polygon of j */
						npclip::polygon_clip(npclip::DIFF, cells[i], sensing[j], cells[i]);
					} else if (quality[i] == quality[j]) {
						/* Use the arbitrary partitioning */
						np::Polygon H = halfplane( seeds[j].center, seeds[i].center, seeds[j].radius );
						npclip::polygon_clip(npclip::DIFF, cells[i], H, cells[i]);
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
		npclip::polygon_clip(npclip::AND, cells[i], region, cells[i]);
	}
}
