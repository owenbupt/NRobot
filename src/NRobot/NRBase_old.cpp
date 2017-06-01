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

#include <cstdio>
#include <cmath>
#include <NPart/NPClip.hpp>
#include "NRBase.hpp"

/*********************************************************/
/******************** Private functions ******************/
/*********************************************************/
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

/*********************************************************/
/******************** Base robot class *******************/
/*********************************************************/



/*********************************************************/
/*************** Mobile Aerial Agent class ***************/
/*********************************************************/
NPFLOAT nr::MAA::get_quality() const {
    NPFLOAT z = this->position.z;
    NPFLOAT zmin = this->zmin;
    NPFLOAT zmax = this->zmax;

    if (z < zmin) {
        return 1;
    } else if (z > zmax) {
        return 0;
    } else {
        return std::pow( std::pow( z-zmin ,2) - std::pow( zmax-zmin ,2) ,2) / std::pow( zmax-zmin ,4);
    }
}

NPFLOAT nr::MAA::get_dquality() const {
    NPFLOAT z = this->position.z;
    NPFLOAT zmin = this->zmin;
    NPFLOAT zmax = this->zmax;

    if (z < zmin || z > zmax) {
        return 0;
    } else {
        return  4*(z-zmin)*(std::pow( z-zmin ,2) - std::pow( zmax-zmin ,2)) / std::pow( zmax-zmin ,4);
    }
}

void nr::MAA::set_quality() {
    this->quality = this->get_quality();
}

void nr::MAA::create_sensing_disk() {
    NPFLOAT r = this->position.z * std::tan(this->view_angle/2);
    this->sensing = np::Circle(this->position, r);
}

void nr::MAA::create_sensing_poly() {
    NPFLOAT r = this->position.z * std::tan(this->view_angle/2);
    this->sensing_poly = np::Polygon( np::Circle(this->position, r) );
}

void nr::MAA::create_cell(const np::Polygon& region, const std::vector<MAA>& robots, size_t i) {
	/* Number of robots (including current) */
	size_t N = robots.size();

	/* Initialize the current MAA's quality and sensing */
	this->set_quality();
	this->create_sensing_disk();
	this->create_sensing_poly();

	/* Initialize the cell of the current MAA to its sensing disk */
	this->cell = this->sensing_poly;

	/* Loop over all other nodes */
	for (size_t j=0; j<N; j++) {
		if (j != i) {
			/* Check for overlapping of the sensing disks */
			if (np::dist(this->sensing.center, robots[j].sensing.center) <=
						this->sensing.radius + robots[j].sensing.radius) {
				/* Compare coverage quality */
				if (this->quality < robots[j].quality) {
					/* Remove the sensing polygon of j */
					npclip::polygon_clip(npclip::DIFF, this->cell, robots[j].sensing_poly, this->cell);
				} else if (this->quality == robots[j].quality) {
					/* Use the arbitrary partitioning */
					np::Polygon H = halfplane( robots[j].sensing.center, this->sensing.center, robots[j].sensing.radius );
					npclip::polygon_clip(npclip::DIFF, this->cell, H, this->cell);
				}
			}
		}
	}

	/* Intersect the cell with the region */
	npclip::polygon_clip(npclip::AND, this->cell, region, this->cell);
}

np::Point nr::MAA::get_velocity(const np::Polygon& region, const std::vector<MAA>& robots, size_t i) {
	np::Point V;

    /* Loop over all external contours of the cell of i */
    for (size_t c=0; c<this->cell.contour.size(); c++) {
        if (!this->cell.is_hole[c]) {
            /* Loop over all edges of the current contour */
            size_t Ne = this->cell.contour[c].size();
            for (size_t k=0; k<Ne; k++) {
                np::Point v1 = this->cell.contour[c][k];
                np::Point v2 = this->cell.contour[c][(k+1) % Ne];

                /* Both v1 and v2 are on the boundary of the sensing disk of i */
                bool both_on_ci = v1.on_dist(this->sensing_poly) && v2.on_dist(this->sensing_poly);

                /* The midpoint of v1 and v2 is on the sensing disk of i */
                bool midpt_on_ci = np::midpoint(v1, v2).on_dist(this->sensing_poly);

                /* Check if the current edge should be integrated over */
				if (both_on_ci && midpt_on_ci) {
                    /* Find the normal vector of the current edge */
                    np::Point n = this->cell.normal(c, k);
                    /* Find the length of the current edge */
                    NPFLOAT l = np::dist(v1, v2);

                    /* Find if the current edge is on the boundary of the cell of another node j */
                    bool both_in_wj = 0;
                    /* Need to keep the index of the neighbor */
                    size_t j;
                    /* Loop over all neighbors */
                    for (j=0; j<robots.size(); j++) {
                        /* Only calculate the control law for neighbors with non-empty cells */
                        if ((j != i) && robots[j].cell.contour.size()) {
                            both_in_wj = v1.in(robots[j].cell) && v2.in(robots[j].cell);
                            if (both_in_wj) {
                                break;
                            }
                        }
                    } /* Neighbor loop */

                    if (both_in_wj) {
                        /* Use the fi-fj integral - Dominant arc */
                        V.x += l * n.x * (this->quality - robots[j].quality);
                        V.y += l * n.y * (this->quality - robots[j].quality);
                        V.z += l * std::tan(this->view_angle/2) * (this->quality - robots[j].quality);
                    } else {
                        /* Use the fi integral - Free Arc */
                        V.x += l * n.x * this->quality;
                        V.y += l * n.y * this->quality;
                        V.z += l * std::tan(this->view_angle/2) * this->quality;
                    }
                }
            } /* Edge of Ci loop */
        }
    } /* Contour loop */

    /* Add the integral over Wi to the z control law */
    V.z += this->cell.area() * this->get_dquality();

    return V;
}
