/*
	Copyright (C) 2016 Sotiris Papatheodorou

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
#include "NRControl.hpp"




np::Point nr::YS_uniform_quality_control(const np::Polygon& region, const std::vector<MAA>& robots, const size_t i, const bool *neighbors) {
    np::Point V;

	// robots[i].cell.print();
	// robots[i].sensing_poly.print();


    /* Loop over all extenral contours of the cell of i */
    for (size_t c=0; c<robots[i].cell.contour.size(); c++) {
        if (!robots[i].cell.is_hole[c]) {
            /* Loop over all edges of the current contour */
            size_t Ne = robots[i].cell.contour[c].size();
            for (size_t k=0; k<Ne; k++) {
                np::Point v1 = robots[i].cell.contour[c][k];
                np::Point v2 = robots[i].cell.contour[c][(k+1) % Ne];

				// printf("------------------ v ------------------\n");
				// printf("x %.20f  y %.20f\n", v1.x, v1.y);
				// printf("x %.20f  y %.20f\n", v2.x, v2.y);
				// printf("------------------ si ------------------\n");
				// robots[i].sensing_poly.print();

                /* Both v1 and v2 are on the boundary of the sensing disk of i */
                bool both_on_ci = v1.on_dist(robots[i].sensing_poly) && v2.on_dist(robots[i].sensing_poly);

				// bool either_on_ci = v1.on_dist(robots[i].sensing_poly) || v2.on_dist(robots[i].sensing_poly);

                /* The midpoint of v1 and v2 is on the sensing disk of i */
                bool midpt_on_ci = np::midpoint(v1, v2).on_dist(robots[i].sensing_poly);

                /* Both v1 and v2 are on the boundary of the region */
                bool both_on_region = v1.on_dist(region) && v2.on_dist(region);
                /* FIX this should NOT be needed */

                // printf("BoCi %d MoCi %d  BoR %d EoCi %d\n", both_on_ci, midpt_on_ci, both_on_region, either_on_ci);

                /* Check if the current edge should be integrated over */
                if (both_on_ci && midpt_on_ci && !both_on_region) {
                    /* Find the normal vector of the current edge */
                    np::Point n = robots[i].cell.normal(c, k);
                    /* Find the length of the current edge */
                    NPFLOAT l = np::dist(v1, v2);
                    // std::cout << n << std::endl;

                    /* Find if the current edge is on the boundary of the cell of another node j */
                    bool both_on_wj;
                    /* Need to keep the index of the neighbor */
                    size_t j;
                    /* Loop over all neighbors */
                    for (j=0; j<robots.size(); j++) {
                        // printf("N %lu - %lu  %d\n",i, j, neighbors[j]);
                        /* Only calculate the control law for neighbors with non-empty cells */
                        if (neighbors[j] && robots[j].cell.contour.size()) {
                            both_on_wj = v1.on(robots[j].cell) && v2.on(robots[j].cell);
                            if (both_on_wj) {
                                break;
                            }
                        }
                    } /* Neighbor loop */

                    // printf("BoWj %d\n", both_on_wj);

                    if (both_on_wj) {
                        // printf("Dominant arc\n");
                        /* Use the fi-fj integral */
                        V.x += l * n.x * (robots[i].quality - robots[j].quality);
                        V.y += l * n.y * (robots[i].quality - robots[j].quality);
                        V.z += l * std::tan(robots[i].view_angle/2) * (robots[i].quality - robots[j].quality);
                    } else {
                        // printf("Free arc\n");
                        /* Use the fi integral */
                        V.x += l * n.x * robots[i].quality;
                        V.y += l * n.y * robots[i].quality;
                        V.z += l * std::tan(robots[i].view_angle/2) * robots[i].quality;
                    }
                }

            } /* Edge of Ci loop */
        }
    } /* Contour loop */

    /* Add the integral over Wi to the z control law */
    V.z += robots[i].cell.area() * robots[i].get_dquality();

    return V;
}
