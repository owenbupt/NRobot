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

#include "NRobot.hpp"
#include "NRPart.hpp"
#if NR_PLOT_AVAILABLE
#include "NRPlot.hpp"
#endif


/*******************************************************/
/********************** MA class ***********************/
/*******************************************************/
nr::MA::MA() {
	this->ID = 0;
	this->sensing_radius = 0;
	this->uncertainty_radius = 0;
	this->communication_radius = 0;
	this->partitioning = nr::PARTITIONING_VORONOI;
	this->control = nr::CONTROL_CENTROID;
	/* The other data members use their default constructors */
}

nr::MA::MA(
	Point& pos,
	double sradius,
	double uradius,
	double cradius
) {
	this->ID = 0;
	this->position = pos;
	this->sensing_radius = sradius;
	this->uncertainty_radius = uradius;
	this->communication_radius = cradius;
	this->partitioning = nr::PARTITIONING_VORONOI;
	this->control = nr::CONTROL_CENTROID;
	/* The other data members use their default constructors */
}

nr::MA::MA(
	Point& pos,
	Orientation& att,
	double sradius,
	double uradius,
	double cradius
) {
	this->ID = 0;
	this->position = pos;
	this->attitude = att;
	this->sensing_radius = sradius;
	this->uncertainty_radius = uradius;
	this->communication_radius = cradius;
	this->partitioning = nr::PARTITIONING_VORONOI;
	this->control = nr::CONTROL_CENTROID;
	/* The other data members use their default constructors */
}




/*******************************************************/
/********************** MAs class **********************/
/*******************************************************/
nr::MAs::MAs() {
	/* All data members use their default constructors */
}

nr::MAs::MAs(
	Points& pos,
	std::vector<double>& sradii,
	std::vector<double>& uradii,
	std::vector<double>& cradii
) {
	/* Number of elements */
	size_t N = pos.size();
	this->resize(N);
	/* Initialize each vector element */
	for (size_t i=0; i<N; i++) {
		this->at(i) = nr::MA( pos[i], sradii[i], uradii[i], cradii[i] );
		/* Set the ID for each element */
		this->at(i).ID = i+1;
	}
}

nr::MAs::MAs(
	Points& pos,
	Orientations& att,
	std::vector<double>& sradii,
	std::vector<double>& uradii,
	std::vector<double>& cradii
) {
	/* Number of elements */
	size_t N = pos.size();
	this->resize(N);
	/* Initialize each vector element */
	for (size_t i=0; i<N; i++) {
		this->at(i) = nr::MA( pos[i], att[i], sradii[i], uradii[i], cradii[i] );
		/* Set the ID for each element */
		this->at(i).ID = i+1;
	}
}






/*******************************************************/
/********************* Non Members *********************/
/*******************************************************/
void nr::info() {
	std::printf("NRobot %d.%d.%d\n",
		NR_VERSION_MAJOR, NR_VERSION_MINOR, NR_VERSION_PATCH);
	std::printf("Copyright (C) 2016-2017 Sotiris Papatheodorou\n");
	std::printf("License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>.\n");
	std::printf("This is free software: you are free to change and redistribute it.\n");
	std::printf("There is NO WARRANTY, to the extent permitted by law.\n\n");

	#if NR_PLOT_AVAILABLE
		SDL_version sdl_linked_version;
		SDL_GetVersion( &sdl_linked_version );
		std::printf("Plotting functionality is available\n");
		std::printf("Using SDL version %d.%d.%d\n\n", sdl_linked_version.major,
			sdl_linked_version.minor, sdl_linked_version.patch);
	#endif
	#if NR_TIME_EXECUTION
		std::printf("Execution is being timed\n\n");
	#endif
}




/*******************************************************/
/***************** Space partitioning ******************/
/*******************************************************/

int nr_cell_voronoi( nr::MA* agent, const nr::Polygon& region ) {
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

int nr_cell_gvoronoi( nr::MA* agent, const nr::Polygon& region ) {
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

int nr_cell_awgvoronoi( nr::MA* agent, const nr::Polygon& region ) {
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

void nr_control_centroid( nr::MA* agent ) {
    /* Vector from the agent to its cell centroid */
    nr::Point v;
    v = nr::centroid( agent->cell ) - agent->position;
    agent->velocity_translational = v;
}

void nr_control_free_arc( nr::MA* agent ) {
    /* Initialize the vector resulting from the integral to zero */
    nr::Point integral_vector;
    /* Number of sensing region edges */
    size_t Ne = agent->sensing.contour[0].size();
    /* Loop over all sensing region edges */
    for (size_t k=0; k<Ne; k++) {
        nr::Point v1 = agent->sensing.contour[0][k];
        nr::Point v2 = agent->sensing.contour[0][(k+1) % Ne];

        /* Check if both edge vertices are inside the agent's cell */
        bool v1_in_cell = nr::in( v1, agent->cell );
        bool v2_in_cell = nr::in( v2, agent->cell );
        if (v1_in_cell && v2_in_cell) {
            /* Add the edge to the integral */
            /* Since external contours are CW, the vector v2-v1 rotated by 90
               degrees points outwards */
            integral_vector += nr::rotate( v2-v1, M_PI/2 );
        }
    }

    agent->velocity_translational = integral_vector;
}

void nr_control_distance( nr::MA* agent ) {
	/* Initialize the control law vector */
	nr::Point control_input;

	/* Loop over all neighbors */
	for (size_t j=0; j<agent->neighbors.size(); j++) {
        /* Examine only neighbors within communication distance */
		if (nr::dist(agent->position, agent->neighbors[j].position) <= agent->communication_radius) {
			nr::Point v = agent->position - agent->neighbors[j].position;
			control_input += 1/(nr::norm(v) + 1) * v / nr::norm(v);
		}
    }

	agent->velocity_translational = control_input;
}




/****** MA ******/
void nr::create_sensing_disk( nr::MA* agent ) {
	nr::Circle C (agent->position, agent->sensing_radius);
	agent->sensing = nr::Polygon( C );
}

void nr::find_neighbors( nr::MA* agent, const nr::MAs& agents ) {
	/* Clear the current neighbor vector */
	agent->neighbors.resize(0);
	/* Loop over all other agents */
	for (size_t j=0; j<agents.size(); j++) {
		if (agent->ID != agents[j].ID) {
			/* Find distance from agent j */
			double d = nr::dist( agent->position, agents[j].position );
			if (d <= agent->communication_radius) {
				/* Add agent j to the neighbor list */
				/* Copy only the required members of agent j to the neighbors vector */
				agent->neighbors.push_back( nr::MA() );
				agent->neighbors.back().ID = agents[j].ID;
				agent->neighbors.back().position = agents[j].position;
				agent->neighbors.back().attitude = agents[j].attitude;
				agent->neighbors.back().sensing_radius = agents[j].sensing_radius;
				agent->neighbors.back().uncertainty_radius = agents[j].uncertainty_radius;
				agent->neighbors.back().communication_radius = agents[j].communication_radius;
			}
		}
	}
}

int nr::compute_cell( nr::MA* agent, const nr::Polygon& region ) {
	int err;
	/* Select the partitioning scheme based on the relevant data member */
	switch (agent->partitioning) {
		case nr::PARTITIONING_VORONOI:
		err = nr_cell_voronoi( agent, region );
		break;

		case nr::PARTITIONING_GVORONOI:
		err = nr_cell_gvoronoi( agent, region );
		break;

		case nr::PARTITIONING_AWGVORONOI:
		err = nr_cell_awgvoronoi( agent, region );
		break;

		default:
		err = nr::ERROR_INVALID_PARTITIONING;
		break;
	}
	return err;
}

void nr::compute_control( nr::MA* agent ) {
	/* Select the control law based on the relevant data member */
	switch (agent->control) {
		case nr::CONTROL_CENTROID:
		nr_control_centroid( agent );
		break;

		case nr::CONTROL_FREE_ARC:
		nr_control_free_arc( agent );
		break;

		case nr::CONTROL_DISTANCE:
		nr_control_distance( agent );
		break;

		default: break;
	}
}

void nr::print( const nr::MA& agent, const bool verbose ) {
	std::printf("MA %lu\n", agent.ID);
	std::printf("  Position: %f %f %f\n",
		agent.position.x, agent.position.y, agent.position.z);
	std::printf("  Attitude: %f %f %f\n",
		agent.attitude.roll, agent.attitude.pitch, agent.attitude.yaw);
	std::printf("  Translational velocity: %f %f %f\n",
		agent.velocity_translational.x, agent.velocity_translational.y, agent.velocity_translational.z);
	std::printf("  Rotational velocity: %f %f %f\n",
		agent.velocity_rotational.roll, agent.velocity_rotational.pitch, agent.velocity_rotational.yaw);
	std::printf("  Sensing radius: %f\n", agent.sensing_radius);
	std::printf("  Uncertainty radius: %f\n", agent.uncertainty_radius);
	std::printf("  Communication radius: %f\n", agent.communication_radius);
	if (verbose) {
		/* Print all polygon vertices if verbose is set */
		std::printf("  Sensing: ");
		nr::print( agent.sensing );
		std::printf("  Cell: ");
		nr::print( agent.cell );
		std::printf("  R-limited cell: ");
		nr::print( agent.rlimited_cell );
	}
	std::printf("  Neighbors:");
	for (size_t j=0; j<agent.neighbors.size(); j++) {
		std::printf(" %lu", agent.neighbors[j].ID);
	}
	std::printf("\n");
	if (verbose) {
		/* Print neighbor members if verbose is set */
		for (size_t j=0; j<agent.neighbors.size(); j++) {
			nr::print(agent.neighbors[j], verbose);
		}
	}
}

void nr::plot_position( const nr::MA& agent ) {
	#if NR_PLOT_AVAILABLE
		nr::plot_point( agent.position );
	#else
		std::printf("Plotting functionality is not available\n");
	#endif
}

void nr::plot_cell( const nr::MA& agent ) {
	#if NR_PLOT_AVAILABLE
		nr::plot_polygon( agent.cell );
	#else
		std::printf("Plotting functionality is not available\n");
	#endif
}

void nr::plot_sensing( const nr::MA& agent ) {
	#if NR_PLOT_AVAILABLE
		nr::plot_polygon( agent.sensing );
	#else
		std::printf("Plotting functionality is not available\n");
	#endif
}

void nr::plot_uncertainty( const nr::MA& agent ) {
	#if NR_PLOT_AVAILABLE
		nr::plot_circle( nr::Circle(agent.position, agent.uncertainty_radius) );
	#else
		std::printf("Plotting functionality is not available\n");
	#endif
}

void nr::plot_communication( const nr::MA& agent ) {
	#if NR_PLOT_AVAILABLE
		nr::plot_circle( nr::Circle(agent.position, agent.communication_radius) );
	#else
		std::printf("Plotting functionality is not available\n");
	#endif
}




/****** MAs ******/
void nr::create_sensing_disks( nr::MAs* agents ) {
	/* Create the sensing disk of each agent */
	for (size_t i=0; i<agents->size(); i++) {
		nr::create_sensing_disk( &(agents->at(i)) );
	}
}

void nr::print( const nr::MAs& agents, const bool verbose ) {
	/* Print each agent */
	for (size_t i=0; i<agents.size(); i++) {
		nr::print( agents[i], verbose );
	}
}

void nr::set_partitioning( nr::MAs* agents, const nr::partitioning_type partitioning ) {
	/* Change the partitioning of each agent */
	for (size_t i=0; i<agents->size(); i++) {
		agents->at(i).partitioning = partitioning;
	}
}

void nr::set_control( nr::MAs* agents, const nr::control_type control ) {
	/* Change the control law of each agent */
	for (size_t i=0; i<agents->size(); i++) {
		agents->at(i).control = control;
	}
}

void nr::plot_positions( const nr::MAs& agents ) {
	#if NR_PLOT_AVAILABLE
		/* Plot the sensing disk of each agent */
		for (size_t i=0; i<agents.size(); i++) {
			nr::plot_point( agents[i].position );
		}
	#else
		std::printf("Plotting functionality is not available\n");
	#endif
}

void nr::plot_cells( const nr::MAs& agents ) {
	#if NR_PLOT_AVAILABLE
		/* Plot the sensing disk of each agent */
		for (size_t i=0; i<agents.size(); i++) {
			nr::plot_polygon( agents[i].cell );
		}
	#else
		std::printf("Plotting functionality is not available\n");
	#endif
}

void nr::plot_sensing( const nr::MAs& agents ) {
	#if NR_PLOT_AVAILABLE
		/* Plot the sensing disk of each agent */
		for (size_t i=0; i<agents.size(); i++) {
			nr::plot_polygon( agents[i].sensing );
		}
	#else
		std::printf("Plotting functionality is not available\n");
	#endif
}

void nr::plot_uncertainty( const nr::MAs& agents ) {
	#if NR_PLOT_AVAILABLE
		/* Plot the sensing disk of each agent */
		for (size_t i=0; i<agents.size(); i++) {
			nr::plot_circle( nr::Circle(agents[i].position, agents[i].uncertainty_radius) );
		}
	#else
		std::printf("Plotting functionality is not available\n");
	#endif
}

void nr::plot_communication( const nr::MAs& agents ) {
	#if NR_PLOT_AVAILABLE
		/* Plot the sensing disk of each agent */
		for (size_t i=0; i<agents.size(); i++) {
			nr::plot_circle( nr::Circle(agents[i].position, agents[i].communication_radius) );
		}
	#else
		std::printf("Plotting functionality is not available\n");
	#endif
}
