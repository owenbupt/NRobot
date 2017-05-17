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

#include "NRPart.hpp"
#include "NRClip.hpp"


/************************************************************/
/********************* Helper functions *********************/
/************************************************************/
nr::Polygon nr_halfplane( const nr::Point& A, const nr::Point& B, const double diam ) {
	/* Initialize halfplane */
	nr::Polygon H;
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
	nr::rotate( &H, theta, true);

	/* Translate halfplane */
	nr::translate( &H, nr::midpoint(A,B) );

	return H;
}

nr::Polygon nr_hyperbola_branch( const nr::Circle& A, const nr::Circle& B, const double diam, const size_t ppb ) {
	/* Initialize hyperbola branch */
	nr::Polygon H;


	if (nr::is_point(A) && nr::is_point(B)) {
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
		double c = nr::dist(A.center, B.center) / 2;
		double b = std::sqrt( c*c - a*a );

		if ( c < a) {
			/* The disks overlap and there is no cell */
			nr::make_empty(&H);
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
	nr::rotate( &H, theta, true);

	/* Translate halfplane */
	nr::translate( &H, nr::midpoint(A.center ,B.center) );

	return H;
}






/**********************************************************/
/********************* Main functions *********************/
/**********************************************************/

void nr::voronoi( const nr::Polygon& region, const nr::Points& seeds, nr::Polygons* cells) {
	/* Number of seeds */
	size_t N = seeds.size();
	/* Initialize the result */
	cells->resize(N);
	/* Region diameter */
	double diam = nr::diameter(region);

	/* Loop over all seed pairs */
	for (size_t i=0; i<N; i++) {
		/* Initialize the cell of i to the region */
		(*cells)[i] = region;
		for (size_t j=0; j<N; j++) {
			if (i != j) {
				/* Create the halfplane containing i with respect to j */
				nr::Polygon h = nr_halfplane( seeds[i], seeds[j], diam );
				/* Intersect the current cell with the halfplane with j */
				nr::polygon_clip( nr::AND, (*cells)[i], h, &((*cells)[i]) );
			}
		}
	}
}

void nr::g_voronoi( const nr::Polygon& region, const nr::Circles& seeds, nr::Polygons* cells, const size_t points_per_branch) {
	/* Number of seeds */
	size_t N = seeds.size();
	/* Initialize the result */
	cells->resize(N);
	/* Region diameter */
	double diam = nr::diameter(region);

	/* Loop over all seed pairs */
	for (size_t i=0; i<N; i++) {
		/* Initialize the cell of i to the region */
		(*cells)[i] = region;
		for (size_t j=0; j<N; j++) {
			if (i != j) {
				/* Create the hyperbola branch containing i with respect to j */
				nr::Polygon h = nr_hyperbola_branch( seeds[i], seeds[j], diam, points_per_branch );
				/* Intersect the current cell with the branch with j */
				nr::polygon_clip( nr::AND, (*cells)[i], h, &((*cells)[i]) );
			}
		}
	}
}

void nr::ys_partitioning( const nr::Polygon& region, const nr::Polygons& seeds, nr::Polygons* cells) {
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
				nr::polygon_clip( nr::DIFF, (*cells)[i], seeds[j], &((*cells)[i]) );
			} else {
				/* If this is node i just find the intersection of the current cell with the pattern of i */
				/* This is done to constrain the cell of i to the region */
				nr::polygon_clip( nr::AND, (*cells)[i], seeds[j], &((*cells)[i]) );
			}
		}
	}

	/* Add the common sensed region */
	nr::make_empty( &((*cells)[N]) );
	Polygon tmpP;
	for (size_t i=0; i<N; i++) {
		/* Find the common sensed part of i */
		nr::polygon_clip( nr::DIFF, seeds[i], (*cells)[i], &(tmpP) );
		/* Add the common sensed part ot i to the total */
		nr::polygon_clip( nr::OR, (*cells)[N], tmpP, &((*cells)[N]) );
	}
	/* Intersect it with the region */
	nr::polygon_clip( nr::AND, (*cells)[N], region, &((*cells)[N]) );
}

void nr::ysuq_partitioning( const nr::Polygon& region, const nr::Circles& seeds, const std::vector<double>& quality, nr::Polygons* cells, bool **neighbors) {
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
	nr::Polygons sensing;
	sensing.resize(N);
	for (size_t i=0; i<N; i++) {
		sensing = nr::Polygons(seeds);
	}

	/* Loop over all nodes */
	for (size_t i=0; i<N; i++) {
		/* Initialize cells to seed polygons */
		(*cells)[i] = sensing[i];

		/* Loop over all other nodes */
		for (size_t j=0; j<N; j++) {
			if (i != j) {
				/* Check for overlapping of the sensing disks */
				if (nr::dist(seeds[i].center, seeds[j].center) <= seeds[i].radius + seeds[j].radius) {
					/* Compare coverage quality */
					if (quality[i] < quality[j]) {
						/* Remove the seed polygon of j */
						nr::polygon_clip(nr::DIFF, (*cells)[i], sensing[j], &((*cells)[i]));
					} else if (quality[i] == quality[j]) {
						/* Use the arbitrary partitioning */
						nr::Polygon H = nr_halfplane( seeds[j].center, seeds[i].center, seeds[j].radius );
						nr::polygon_clip(nr::DIFF, (*cells)[i], H, &((*cells)[i]));
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
		nr::polygon_clip(nr::AND, (*cells)[i], region, &((*cells)[i]));
	}
}
