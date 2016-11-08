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

    /* Loop over all external contours of the cell of i */
    for (size_t c=0; c<robots[i].cell.contour.size(); c++) {
        if (!robots[i].cell.is_hole[c]) {
            /* Loop over all edges of the current contour */
            size_t Ne = robots[i].cell.contour[c].size();
            for (size_t k=0; k<Ne; k++) {
                np::Point v1 = robots[i].cell.contour[c][k];
                np::Point v2 = robots[i].cell.contour[c][(k+1) % Ne];

                /* Both v1 and v2 are on the boundary of the sensing disk of i */
                bool both_on_ci = v1.on_dist(robots[i].sensing_poly) && v2.on_dist(robots[i].sensing_poly);

                /* The midpoint of v1 and v2 is on the sensing disk of i */
                bool midpt_on_ci = np::midpoint(v1, v2).on_dist(robots[i].sensing_poly);

                /* Check if the current edge should be integrated over */
				if (both_on_ci && midpt_on_ci) {
                    /* Find the normal vector of the current edge */
                    np::Point n = robots[i].cell.normal(c, k);
                    /* Find the length of the current edge */
                    NPFLOAT l = np::dist(v1, v2);

                    /* Find if the current edge is on the boundary of the cell of another node j */
                    bool both_in_wj = 0;
                    /* Need to keep the index of the neighbor */
                    size_t j;
                    /* Loop over all neighbors */
                    for (j=0; j<robots.size(); j++) {
                        /* Only calculate the control law for neighbors with non-empty cells */
                        if ((j != i) && neighbors[j] && robots[j].cell.contour.size()) {
                            both_in_wj = v1.in(robots[j].cell) && v2.in(robots[j].cell);
                            if (both_in_wj) {
                                break;
                            }
                        }
                    } /* Neighbor loop */

                    if (both_in_wj) {
                        /* Use the fi-fj integral - Dominant arc */
                        V.x += l * n.x * (robots[i].quality - robots[j].quality);
                        V.y += l * n.y * (robots[i].quality - robots[j].quality);
                        V.z += l * std::tan(robots[i].view_angle/2) * (robots[i].quality - robots[j].quality);
                    } else {
                        /* Use the fi integral - Free Arc */
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
