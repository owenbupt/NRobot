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

#include "NRControl.hpp"
#include "NRPart.hpp"

/*******************************************************/
/***************** Space partitioning ******************/
/*******************************************************/
int nr::cell_voronoi( nr::MA* agent, const nr::Polygon& region ) {
    /* Create a vector containing all neighbor positions */
    nr::Points positions;
    for (size_t j=0; j<agent->neighbors.size(); j++) {
        positions.push_back( agent->neighbors[j].position );
    }
    /* Add the position of the current agent to the vector */
    positions.push_back( agent->position );

    /* Compute voronoi cell */
    nr::voronoi_cell( region, positions, positions.size()-1, &(agent->cell) );

    return nr::SUCCESS;
}




/*******************************************************/
/******************** Control laws *********************/
/*******************************************************/
void nr::control_centroid( MA* agent ) {
    /* Vector from the agent to its cell centroid */
    nr::Point v;
    v = nr::centroid( agent->cell ) - agent->position;
    agent->velocity_translational = v;
}
