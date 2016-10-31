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
#include "NRControl.hpp"

np::Point nr::YS_uniform_quality_control(const np::Polygon& region, const int i, const np::Polygons& cells, const std::vector<double>& quality, const bool *neighbors) {
    np::Point V;

    /* Loop over all neighbors */
    for (size_t j=0; j<cells.size(); j++) {
        /* Only calculate the control law for neighbors with non-empty cells */
        if (neighbors[j] && cells[j].contour.size()) {
            /* Loop over all extenral contours of the cell of i */
            for (size_t c=0; c<cells[i].contour.size(); c++){
                if (!cells[i].is_hole[c]) {
                    /* Loop over all edges of the current contour */
                    size_t Ne = cells[i].contour[c].size();
                    for (size_t k=0; k<Ne; k++) {
                        np::Point v1 = cells[i].contour[c][k];
                        np::Point v2 = cells[i].contour[c][(k+1) % Ne];

                        
                    } /* Edge loop */
                }
            } /* Contour loop */
        }
    } /* Neighbor loop */

    return V;
}
