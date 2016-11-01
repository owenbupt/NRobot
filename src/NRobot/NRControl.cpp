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

    /* Loop over all neighbors */
    for (size_t j=0; j<robots.size(); j++) {
        /* Only calculate the control law for neighbors with non-empty cells */
        if (neighbors[j] && robots[j].cell.contour.size()) {
            /* Loop over all extenral contours of the cell of i */
            for (size_t c=0; c<robots[i].cell.contour.size(); c++){
                if (!robots[i].cell.is_hole[c]) {
                    /* Loop over all edges of the current contour */
                    size_t Ne = robots[i].cell.contour[c].size();
                    for (size_t k=0; k<Ne; k++) {
                        np::Point v1 = robots[i].cell.contour[c][k];
                        np::Point v2 = robots[i].cell.contour[c][(k+1) % Ne];

                        /* Both v1 and v2 are on the boundary of the cell of i */
                        bool both_on_wi = v1.on(robots[i].cell) && v2.on(robots[i].cell);

                        /* One v1 and v2 is on the boundary of the cell of i and the other on the boundary of the region */
                        bool on_wi_and_b = (v1.on(robots[i].cell) && v2.on(region)) || (v2.on(robots[i].cell) && v1.on(region));

                        /* The midpoint of v1 and v2 is on the boundary of the cell of i */
                        bool midpt_on_wi = midpoint(v1, v2).on(robots[i].cell);

                        /* Check if the current edge should be integrated over */
                        if ((both_on_wi || on_wi_and_b) && midpt_on_wi) {
                            /* Find if the current edge is on the boundary of the cell of another node j */
                            bool both_on_wj = v1.on(robots[j].cell) && v2.on(robots[j].cell);
                            if (both_on_wj) {
                                /* Use the fi-fj integral */
                                V.x += 0;
                                V.y += 0;
                                V.z += std::tan(robots[i].view_angle/2) * (robots[i].quality - robots[j].quality);
                            } else {
                                /* Use the fi integral */
                                V.x += 0;
                                V.y += 0;
                                V.z += std::tan(robots[i].view_angle/2) * robots[i].quality;
                            }
                        }

                    } /* Edge of Ci loop */
                }
            } /* Contour loop */
        }
    } /* Neighbor loop */

    /* Add the integral over Wi to the z control law */
    V.z += robots[i].cell.area() * robots[i].get_dquality();

    return V;
}
