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
    /* Create a vector containing all agent positions */
    nr::Points positions;
    /* Add the position of the current agent to the vector */
    positions.push_back( agent->position );
    /* Add the positions of the current agent's neighbors to the vector */
    for (size_t j=0; j<agent->neighbors.size(); j++) {
        positions.push_back( agent->neighbors[j].position );
    }

    /* Compute voronoi cell */
    int err = nr::voronoi_cell( region, positions, 0, &(agent->cell) );
    if (err) {
        std::printf("Clipping operation returned error %d\n", err);
        return nr::ERROR_PARTITIONING_FAILED;
    }

    return nr::SUCCESS;
}

int nr::cell_gvoronoi( nr::MA* agent, const nr::Polygon& region ) {
    /* Create a vector containing all agent positioning uncertainty disks */
    nr::Circles uncert_disks;
    /* Add the positioning uncertainty of the current agent to the vector */
    uncert_disks.push_back( nr::Circle(agent->position, agent->uncertainty_radius) );
    /* Add the positioning uncertainty of the current agent's neighbors to the vector */
    for (size_t j=0; j<agent->neighbors.size(); j++) {
        uncert_disks.push_back( nr::Circle(agent->neighbors[j].position, agent->neighbors[j].uncertainty_radius) );
    }

    /* Compute voronoi cell */
    int err = nr::g_voronoi_cell( region, uncert_disks, 0, &(agent->cell) );
    if (err) {
        std::printf("Clipping operation returned error %d\n", err);
        return nr::ERROR_PARTITIONING_FAILED;
    }

    return nr::SUCCESS;
}

int nr::cell_awgvoronoi( nr::MA* agent, const nr::Polygon& region ) {
    /* Create vectors containing all agent positioning uncertainty disks and sensing radii */
    nr::Circles uncert_disks;
    std::vector<double> sensing_radii;
    /* Add the positioning uncertainty and sensing radius of the current agent to the vector */
    uncert_disks.push_back( nr::Circle(agent->position, agent->uncertainty_radius) );
    sensing_radii.push_back( agent->sensing_radius );
    /* Add the positioning uncertainty and sensing radii of the current agent's neighbors to the vector */
    for (size_t j=0; j<agent->neighbors.size(); j++) {
        uncert_disks.push_back( nr::Circle(agent->neighbors[j].position, agent->neighbors[j].uncertainty_radius) );
        sensing_radii.push_back( agent->neighbors[j].sensing_radius );
    }

    /* Compute voronoi cell */
    int err = nr::awg_voronoi_cell( region, uncert_disks, sensing_radii, 0, &(agent->cell) );
    if (err) {
        std::printf("Clipping operation returned error %d\n", err);
        return nr::ERROR_PARTITIONING_FAILED;
    }

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

void nr::control_free_arc( nr::MA* agent ) {
    /* */
    nr::Point v;

    /* Check all sensing region edges. if they are inside the cell, integrate */

    agent->velocity_translational = v;
}
