/*
 *  Copyright (C) 2016-2017 Sotiris Papatheodorou
 *
 *  This file is part of NRobot.
 *
 *  NRobot is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  NRobot is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with NRobot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "NRobot.hpp"


/*******************************************************/
/********************** MA class ***********************/
/*******************************************************/
nr::MA::MA() {
	this->ID = 0;
	/* Agent parameters */
	this->sensing_radius = 0;
	this->communication_radius = 0;
	this->position_uncertainty = 0;
	this->attitude_uncertainty = 0;
	this->relaxed_sensing_quality = 0;
	this->save_unassigned_sensing = false;
	/* Control */
	this->partitioning = nr::PARTITIONING_VORONOI;
	this->control = nr::CONTROL_CENTROID;
	this->avoidance = nr::AVOIDANCE_DISK_BISECTOR;
	this->control_input = std::vector<double> (6,0);
	this->control_input_gains = std::vector<double> (6,1);
	/* Dynamics and simulation */
	this->dynamics = nr::DYNAMICS_SI_GROUND_XY;
	this->time_step = 0.01;
	/* The other data members use their default constructors */
}

nr::MA::MA(
	nr::Point& pos,
	double time_step,
	double sradius,
	double uradius,
	double cradius
) {
	this->ID = 0;
	/* State */
	this->position = pos;
	/* Agent parameters */
	this->sensing_radius = sradius;
	this->communication_radius = cradius;
	this->position_uncertainty = uradius;
	this->attitude_uncertainty = 0;
	this->relaxed_sensing_quality = 0;
	this->save_unassigned_sensing = false;
	/* Control */
	this->partitioning = nr::PARTITIONING_VORONOI;
	this->control = nr::CONTROL_CENTROID;
	this->avoidance = nr::AVOIDANCE_DISK_BISECTOR;
	this->control_input = std::vector<double> (6,0);
	this->control_input_gains = std::vector<double> (6,1);
	/* Dynamics and simulation */
	this->dynamics = nr::DYNAMICS_SI_GROUND_XY;
	this->time_step = time_step;
	/* The other data members use their default constructors */
}

nr::MA::MA(
	nr::Point& pos,
	nr::Orientation& att,
	double time_step,
	double sradius,
	double uradius,
	double cradius
) {
	this->ID = 0;
	/* State */
	this->position = pos;
	this->attitude = att;
	/* Agent parameters */
	this->sensing_radius = sradius;
	this->communication_radius = cradius;
	this->position_uncertainty = uradius;
	this->attitude_uncertainty = 0;
	this->relaxed_sensing_quality = 0;
	this->save_unassigned_sensing = false;
	/* Control */
	this->partitioning = nr::PARTITIONING_VORONOI;
	this->control = nr::CONTROL_CENTROID;
	this->avoidance = nr::AVOIDANCE_DISK_BISECTOR;
	this->control_input = std::vector<double> (6,0);
	this->control_input_gains = std::vector<double> (6,1);
	/* Dynamics and simulation */
	this->dynamics = nr::DYNAMICS_SI_GROUND_XY;
	this->time_step = time_step;
	/* The other data members use their default constructors */
}




/*******************************************************/
/********************** MAs class **********************/
/*******************************************************/
nr::MAs::MAs() {
	/* All data members use their default constructors */
}

nr::MAs::MAs(
	nr::Points& pos,
	double time_step
) {
	/* Number of elements */
	size_t N = pos.size();
	this->resize(N);
	/* Initialize each vector element */
	for (size_t i=0; i<N; i++) {
		this->at(i) = nr::MA( pos[i], time_step );
		/* Set the ID for each element */
		this->at(i).ID = i+1;
	}
}

nr::MAs::MAs(
	nr::Points& pos,
	double time_step,
	std::vector<double>& sradii,
	std::vector<double>& uradii,
	std::vector<double>& cradii
) {
	/* Number of elements */
	size_t N = pos.size();
	this->resize(N);
	/* Initialize each vector element */
	for (size_t i=0; i<N; i++) {
		this->at(i) = nr::MA( pos[i], time_step, sradii[i], uradii[i], cradii[i] );
		/* Set the ID for each element */
		this->at(i).ID = i+1;
	}
}

nr::MAs::MAs(
	nr::Points& pos,
	nr::Orientations& att,
	double time_step
) {
	/* Number of elements */
	size_t N = pos.size();
	this->resize(N);
	/* Initialize each vector element */
	for (size_t i=0; i<N; i++) {
		this->at(i) = nr::MA( pos[i], att[i], time_step );
		/* Set the ID for each element */
		this->at(i).ID = i+1;
	}
}

nr::MAs::MAs(
	nr::Points& pos,
	nr::Orientations& att,
	double time_step,
	std::vector<double>& sradii,
	std::vector<double>& uradii,
	std::vector<double>& cradii
) {
	/* Number of elements */
	size_t N = pos.size();
	this->resize(N);
	/* Initialize each vector element */
	for (size_t i=0; i<N; i++) {
		this->at(i) = nr::MA( pos[i], att[i], time_step, sradii[i], uradii[i], cradii[i] );
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

	/* Compute r-limited cell */
    err = nr::polygon_clip( nr::AND, agent->cell, agent->sensing,
	    &(agent->rlimited_cell) );
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
    uncert_disks.push_back( nr::Circle(agent->position, agent->position_uncertainty) );
    /* Add the positioning uncertainty of the current agent's neighbors to the vector */
    for (size_t j=0; j<agent->neighbors.size(); j++) {
        uncert_disks.push_back( nr::Circle(agent->neighbors[j].position, agent->neighbors[j].position_uncertainty) );
    }

    /* Compute voronoi cell */
    int err = nr::g_voronoi_cell( region, uncert_disks, 0, &(agent->cell) );
    if (err) {
        std::printf("Clipping operation returned error %d\n", err);
        return nr::ERROR_PARTITIONING_FAILED;
    }

	/* Compute r-limited cell */
    err = nr::polygon_clip( nr::AND, agent->cell, agent->sensing,
	    &(agent->rlimited_cell) );
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
    uncert_disks.push_back( nr::Circle(agent->position, agent->position_uncertainty) );
    sensing_radii.push_back( agent->sensing_radius );
    /* Add the positioning uncertainty and sensing radii of the current agent's neighbors to the vector */
    for (size_t j=0; j<agent->neighbors.size(); j++) {
        uncert_disks.push_back( nr::Circle(agent->neighbors[j].position, agent->neighbors[j].position_uncertainty) );
        sensing_radii.push_back( agent->neighbors[j].sensing_radius );
    }

    /* Compute voronoi cell */
    int err = nr::awg_voronoi_cell( region, uncert_disks, sensing_radii, 0, &(agent->cell) );
    if (err) {
        std::printf("Clipping operation returned error %d\n", err);
        return nr::ERROR_PARTITIONING_FAILED;
    }

	/* Compute r-limited cell */
    err = nr::polygon_clip( nr::AND, agent->cell, agent->sensing,
	    &(agent->rlimited_cell) );
    if (err) {
        std::printf("Clipping operation returned error %d\n", err);
        return nr::ERROR_PARTITIONING_FAILED;
    }

    return nr::SUCCESS;
}

int nr_cell_anisotropic_partitioning( nr::MA* agent, const nr::Polygon& region ) {
	/* Create vectors containing all sensing regions. */
	nr::Polygons sensing;
	/* Add the sensing patterns fo the current agent. */
	sensing.push_back( agent->sensing );

	/* Add the sensing patterns of its neighbors. */
	for (size_t j=0; j<agent->neighbors.size(); j++) {
		sensing.push_back( agent->neighbors[j].sensing );
	}

	int err;
	if (agent->save_unassigned_sensing) {
		err = nr::anisotropic_partitioning_cell( region, sensing, 0,
		    &(agent->cell), &(agent->unassigned_sensing) );
	} else {
		err = nr::anisotropic_partitioning_cell( region, sensing, 0,
		    &(agent->cell) );
	}

	if (err) {
        std::printf("Clipping operation returned error %d\n", err);
        return nr::ERROR_PARTITIONING_FAILED;
    }

    return nr::SUCCESS;
}

int nr_cell_au_partitioning( nr::MA* agent, const nr::Polygon& region ) {
	/* Create vectors containing all guaranteed, relaxed and total sensing
	   regions. */
	nr::Polygons guaranteed_sensing, relaxed_sensing, total_sensing;
	/* Add the sensing patterns fo the current agent. */
	guaranteed_sensing.push_back( agent->guaranteed_sensing );
	relaxed_sensing.push_back( agent->relaxed_sensing );
	total_sensing.push_back( agent->total_sensing );

	/* Add the sensing patterns of its neighbors. */
	for (size_t j=0; j<agent->neighbors.size(); j++) {
		guaranteed_sensing.push_back( agent->neighbors[j].guaranteed_sensing );
		relaxed_sensing.push_back( agent->neighbors[j].relaxed_sensing );
		total_sensing.push_back( agent->neighbors[j].total_sensing );
	}

	int err;
	if (agent->save_unassigned_sensing) {
		err = nr::au_partitioning_cell( region, guaranteed_sensing,
		    relaxed_sensing, total_sensing, agent->relaxed_sensing_quality, 0,
		    &(agent->cell), &(agent->unassigned_sensing) );
	} else {
		err = nr::au_partitioning_cell( region, guaranteed_sensing,
		    relaxed_sensing, total_sensing, agent->relaxed_sensing_quality, 0,
		    &(agent->cell) );
	}

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
	agent->control_input[0] = v.x;
	agent->control_input[1] = v.y;
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

	agent->control_input[0] = integral_vector.x;
	agent->control_input[1] = integral_vector.y;
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

	agent->control_input[0] = control_input.x;
	agent->control_input[1] = control_input.y;
}

void nr_control_anisotropic( nr::MA* agent ) {
	/* Loop over all vertices of sensing. If it is also a vertex of the cell,
	   then add it to the integral. */

	/* Initialize the vector resulting from the integral to zero. */
	nr::Point planar_integral;
	double angle_integral = 0;

	/* Loop over all sensing contours. */
	size_t Nc = agent->sensing.contour.size();
	for (size_t c=0; c<Nc; c++) {
		/* Loop over all edges of the contour. */
		size_t Ne = agent->sensing.contour[c].size();
		for (size_t v=0; v<Ne; v++) {
			/* Get the edge vertices. */
			nr::Point v1 = agent->sensing.contour[c][v];
			nr::Point v2 = agent->sensing.contour[c][(v+1) % Ne];
			/* Check if they are both on the boundary of the cell. */
			bool on_v1 = nr::is_vertex_of(v1, agent->cell);
			bool on_v2 = nr::is_vertex_of(v2, agent->cell);
			if (on_v1 && on_v2) {
				/* Calculate the normal vector. External contours are CW so
				   rotate the vertex by 90 degrees. */
				nr::Point n = nr::rotate( v2-v1, M_PI/2 );
				/* Add it to the integrals */
				planar_integral += n;
				angle_integral += nr::dot( nr::rotate( nr::midpoint(v1,v2)
				    - agent->position, 0.5*M_PI ), n);
			}
		}
	}

	agent->control_input[0] = planar_integral.x;
	agent->control_input[1] = planar_integral.y;
	agent->control_input[2] = angle_integral;
}

void nr_control_au( nr::MA* agent ) {
	/* Loop over all vertices of sensing. If it is also a vertex of the cell,
	   then add it to the integral. */

	/* Select the sensing region based on the value of the relaxed sensing
	   quality. */
	nr::Polygon sensing;
	if (agent->relaxed_sensing_quality == 0) {
		sensing = agent->guaranteed_sensing;
	} else if (agent->relaxed_sensing_quality == 1) {
		sensing = agent->total_sensing;
	} else {
		/* Not implemented yet */
		return;
	}

	/* Initialize the vector resulting from the integral to zero. */
	nr::Point planar_integral;
	double angle_integral = 0;

	/* Loop over all sensing contours. */
	size_t Nc = sensing.contour.size();
	for (size_t c=0; c<Nc; c++) {
		/* Loop over all edges of the contour. */
		size_t Ne = sensing.contour[c].size();
		for (size_t v=0; v<Ne; v++) {
			/* Get the edge vertices. */
			nr::Point v1 = sensing.contour[c][v];
			nr::Point v2 = sensing.contour[c][(v+1) % Ne];
			/* Check if they are both on the boundary of the cell. */
			bool on_v1 = nr::is_vertex_of(v1, agent->cell);
			bool on_v2 = nr::is_vertex_of(v2, agent->cell);
			if (on_v1 && on_v2) {
				/* Calculate the normal vector. External contours are CW so
				   rotate the vertex by 90 degrees. */
				nr::Point n = nr::rotate( v2-v1, M_PI/2 );
				/* Add it to the integrals */
				planar_integral += n;
				angle_integral += nr::dot( nr::rotate( nr::midpoint(v1,v2)
				    - agent->position, 0.5*M_PI ), n);
			}
		}
	}

	agent->control_input[0] = planar_integral.x;
	agent->control_input[1] = planar_integral.y;
	agent->control_input[2] = angle_integral;
}




/****** MA ******/
void nr::create_sensing_disk(
	nr::MA* agent
) {
	nr::Circle C (agent->position, agent->sensing_radius);
	agent->sensing = nr::Polygon( C );
}

void nr::find_neighbors(
	nr::MA* agent,
	const nr::MAs& agents
) {
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
				agent->neighbors.back().communication_radius = agents[j].communication_radius;
				agent->neighbors.back().position_uncertainty = agents[j].position_uncertainty;
				agent->neighbors.back().attitude_uncertainty = agents[j].attitude_uncertainty;
				agent->neighbors.back().sensing = agents[j].sensing;
				agent->neighbors.back().guaranteed_sensing = agents[j].guaranteed_sensing;
				agent->neighbors.back().relaxed_sensing = agents[j].relaxed_sensing;
				agent->neighbors.back().total_sensing = agents[j].total_sensing;
				agent->neighbors.back().dynamics = agents[j].dynamics;
				agent->neighbors.back().partitioning = agents[j].partitioning;
				agent->neighbors.back().control = agents[j].control;
			}
		}
	}
}

void nr::simulate_dynamics(
	nr::MA* agent
) {
	switch (agent->dynamics) {
		default:
		case DYNAMICS_SI_GROUND_XY:
		agent->position.x += agent->time_step *
		    agent->control_input_gains[0] * agent->control_input[0];
		agent->position.y += agent->time_step *
		    agent->control_input_gains[1] * agent->control_input[1];
		break;

		case DYNAMICS_SI_GROUND_XYy:
		agent->position.x += agent->time_step *
		    agent->control_input_gains[0] * agent->control_input[0];
		agent->position.y += agent->time_step *
		    agent->control_input_gains[1] * agent->control_input[1];
		agent->attitude.yaw += agent->time_step *
		    agent->control_input_gains[2] * agent->control_input[2];
		break;

		case DYNAMICS_SI_AIR_XYZ:
		agent->position.x += agent->time_step *
		    agent->control_input_gains[0] * agent->control_input[0];
		agent->position.y += agent->time_step *
		    agent->control_input_gains[1] * agent->control_input[1];
		agent->position.z += agent->time_step *
		    agent->control_input_gains[2] * agent->control_input[2];
		break;

		case DYNAMICS_SI_AIR_XYZy:
		agent->position.x += agent->time_step *
		    agent->control_input_gains[0] * agent->control_input[0];
		agent->position.y += agent->time_step *
		    agent->control_input_gains[1] * agent->control_input[1];
		agent->position.z += agent->time_step *
		    agent->control_input_gains[2] * agent->control_input[2];
		agent->attitude.yaw += agent->time_step *
		    agent->control_input_gains[3] * agent->control_input[3];
		break;
	}
}

int nr::compute_base_sensing_patterns(
    nr::MA* agent
) {
	/* Only if the agent has position or orientation uncertainty. */
	if (agent->position_uncertainty + agent->attitude_uncertainty > 0) {
		int err;
		int ATT_POINTS = 50;
		int RAD_POINTS = 5;
		int ANGLE_POINTS = 60;

		nr::Polygon gs_rot, gs_tr, ts_rot, ts_tr;

		/* Create uncertainty vectors */
		/* Attitude vector, create equaly spaced vector of ATT_POINTS values */
		std::vector<double> attitude_uncert_vector =
			nr::linspace( -agent->attitude_uncertainty, agent->attitude_uncertainty, ATT_POINTS );

		/* Radius vector, create equaly spaced vector of RAD_POINTS values */
		std::vector<double> radius_uncert_vector =
			nr::linspace( 0, agent->position_uncertainty, RAD_POINTS+1 );
		/* Delete the zero radius */
		radius_uncert_vector.erase(radius_uncert_vector.begin());

		/* Angle vector, create equaly spaced vector of ANGLE_POINTS values */
		std::vector<double> angle_vector =
			nr::linspace( 0, 2*M_PI, ANGLE_POINTS+1 );
		/* Delete the 2pi angle */
		angle_vector.pop_back();


		/* Guaranteed sensing computation */
		/* Intersect the base sensing for all uncertainty values */
		/* Rotation */
		/* Initialize the guaranteed rotatation base sensing to the base sensing */
		gs_rot = agent->base_sensing;
		/* Loop over all possible orientations */
		for (size_t l=0; l<attitude_uncert_vector.size(); l++) {
			if (!nr::is_empty( gs_rot )) {
				/* Create a temporary rotated copy of the base sensing */
				nr::Polygon tmp_poly = agent->base_sensing;
				nr::rotate( &tmp_poly, attitude_uncert_vector[l], true );
				/* Intersect the temporary polygon with the guaranteed rotatation base sensing */
				int err = nr::polygon_clip( nr::AND, gs_rot, tmp_poly, &gs_rot );
				if (err) {
					std::printf("Clipping operation returned error %d\n", err);
					return nr::ERROR_CLIPPING_FAILED;
				}
			} else {
				/* If the guaranteed rotatation base sensing is empty, stop */
				break;
			}
		}
		/* Translation */
		/* Initialize the guaranteed translation base sensing to the base sensing */
		gs_tr = agent->base_sensing;
		for (size_t l=0; l<radius_uncert_vector.size(); l++) {
			for (size_t k=0; k<angle_vector.size(); k++) {
				if (!nr::is_empty( gs_tr )) {
					/* Create a temporary translation vector */
					nr::Point tmp_vector = nr::pol2cart( nr::Point(radius_uncert_vector[l], angle_vector[k]) );
					/* Create a temporary translated copy of the guaranteed rotatation base sensing */
					nr::Polygon tmp_poly = gs_rot;
					nr::translate( &tmp_poly, tmp_vector );
					/* Intersect the temporary polygon with the guaranteed translation base sensing */
					int err = nr::polygon_clip( nr::AND, gs_tr, tmp_poly, &gs_tr );
					if (err) {
						std::printf("Clipping operation returned error %d\n", err);
						return nr::ERROR_CLIPPING_FAILED;
					}
				} else {
					std::printf("empty\n");
					break;
				}
			}
		}
		/* Set the base guaranteed sensing pattern on the agent */
		agent->base_guaranteed_sensing = gs_tr;

		/* Total sensing computation */
		/* Union of the base sensing for all uncertainty values */
		/* Rotation */
		/* Initialize the guaranteed rotatation base sensing to the base sensing */
		ts_rot = agent->base_sensing;
		/* Loop over all possible orientations */
		for (size_t l=0; l<attitude_uncert_vector.size(); l++) {
			if (!nr::is_empty( ts_rot )) {
				/* Create a temporary rotated copy of the base sensing */
				nr::Polygon tmp_poly = agent->base_sensing;
				nr::rotate( &tmp_poly, attitude_uncert_vector[l], true );
				/* Intersect the temporary polygon with the guaranteed rotatation base sensing */
				int err = nr::polygon_clip( nr::OR, ts_rot, tmp_poly, &ts_rot );
				if (err) {
					std::printf("Clipping operation returned error %d\n", err);
					return nr::ERROR_CLIPPING_FAILED;
				}
			} else {
				/* If the guaranteed rotatation base sensing is empty, stop */
				break;
			}
		}
		/* Translation */
		/* Initialize the guaranteed translation base sensing to the base sensing */
		ts_tr = agent->base_sensing;
		for (size_t l=0; l<radius_uncert_vector.size(); l++) {
			for (size_t k=0; k<angle_vector.size(); k++) {
				if (!nr::is_empty( ts_tr )) {
					/* Create a temporary translation vector */
					nr::Point tmp_vector = nr::pol2cart( nr::Point(radius_uncert_vector[l], angle_vector[k]) );
					/* Create a temporary translated copy of the guaranteed rotatation base sensing */
					nr::Polygon tmp_poly = ts_rot;
					nr::translate( &tmp_poly, tmp_vector );
					/* Intersect the temporary polygon with the guaranteed translation base sensing */
					int err = nr::polygon_clip( nr::OR, ts_tr, tmp_poly, &ts_tr );
					if (err) {
						std::printf("Clipping operation returned error %d\n", err);
						return nr::ERROR_CLIPPING_FAILED;
					}
				} else {
					std::printf("empty\n");
					break;
				}
			}
		}
		/* Set the base guaranteed sensing pattern on the agent */
		agent->base_total_sensing = ts_tr;

		/* Relaxed sensing computation */
		/* Subtract the guaranteed sensing from the total sensing */
		err = nr::polygon_clip( nr::DIFF,
			agent->base_total_sensing,
			agent->base_guaranteed_sensing,
			&(agent->base_relaxed_sensing) );
		if (err) {
			std::printf("Clipping operation returned error %d\n", err);
			return nr::ERROR_CLIPPING_FAILED;
		}
	}

	/* Fix contour orientation */
	nr::fix_orientation( &(agent->base_guaranteed_sensing) );
	nr::fix_orientation( &(agent->base_relaxed_sensing) );
	nr::fix_orientation( &(agent->base_total_sensing) );

	return nr::SUCCESS;
}

void nr::update_sensing_patterns(
    nr::MA* agent
) {
	/* Sensing */
	agent->sensing = agent->base_sensing;
	nr::rotate( &(agent->sensing), agent->attitude.yaw, true );
	nr::translate( &(agent->sensing), agent->position );
	/* Guaranteed Sensing */
	if ( !nr::is_empty(agent->base_guaranteed_sensing) ) {
		agent->guaranteed_sensing = agent->base_guaranteed_sensing;
		nr::rotate( &(agent->guaranteed_sensing), agent->attitude.yaw, true );
		nr::translate( &(agent->guaranteed_sensing), agent->position );
	}
	/* Relaxed Sensing */
	if ( !nr::is_empty(agent->base_relaxed_sensing) ) {
		agent->relaxed_sensing = agent->base_relaxed_sensing;
		nr::rotate( &(agent->relaxed_sensing), agent->attitude.yaw, true );
		nr::translate( &(agent->relaxed_sensing), agent->position );
	}
	/* Total Sensing */
	if ( !nr::is_empty(agent->base_total_sensing) ) {
		agent->total_sensing = agent->base_total_sensing;
		nr::rotate( &(agent->total_sensing), agent->attitude.yaw, true );
		nr::translate( &(agent->total_sensing), agent->position );
	}
}

int nr::compute_cell(
	nr::MA* agent,
	const nr::Polygon& region
) {
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

		case nr::PARTITIONING_ANISOTROPIC:
		err = nr_cell_anisotropic_partitioning( agent, region );
		break;

		case nr::PARTITIONING_ANISOTROPIC_UNCERTAINTY:
		err = nr_cell_au_partitioning( agent, region );
		break;

		default:
		err = nr::ERROR_INVALID_PARTITIONING;
		std::printf("Invalid partitioning selected.\n");
		break;
	}
	return err;
}

void nr::compute_control(
	nr::MA* agent
) {
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

		case nr::CONTROL_ANISOTROPIC:
		nr_control_anisotropic( agent );
		break;

		case nr::CONTROL_ANISOTROPIC_UNCERTAINTY:
		nr_control_au( agent );
		break;

		default:
		std::printf("Invalid control law selected.\n");
		break;
	}
}

void nr::ensure_collision_avoidance(
    nr::MA* agent
) {
	/* Collision avoidance tolerance. */
	double e = 0.001;
	/* Loop over all neighbors. */
	for (size_t j=0; j<agent->neighbors.size(); j++) {
		/* Find the distance from the neighbor. */
		double d = nr::dist( agent->position, agent->neighbors[j].position );
		/* If the distance plus the tolerance is equal or less than the sum of
		   the positioning uncertainties of the agents, they might collide. */
		if (d+e <= agent->position_uncertainty +
		agent->neighbors[j].position_uncertainty) {
			/* The translational control input. */
			nr::Point tmp_ctrl_input
			(agent->control_input[0], agent->control_input[1]);
			/* Vector from the agent to its neighbor. */
			nr::Point v = agent->neighbors[j].position - agent->position;
			/* Check if the control input points towards the neighbor. */
			if (nr::dot( tmp_ctrl_input, v ) > 0) {
				/* Create a vector with the direction of the bisector. */
				nr::Point pb = nr::rotate(
				agent->position-agent->neighbors[j].position, M_PI/2 );
				/* Project the control input on the perpendicular bisector. */

				tmp_ctrl_input = nr::projection( tmp_ctrl_input, pb );

				agent->control_input[0] = tmp_ctrl_input.x;
				agent->control_input[1] = tmp_ctrl_input.y;
			}
		}
	}
}

double nr::calculate_objective(
   nr::MA& agent
) {
	double H = 0;

	switch (agent.partitioning) {

		case nr::PARTITIONING_VORONOI:
		case nr::PARTITIONING_GVORONOI:
		case nr::PARTITIONING_AWGVORONOI:
		if ((agent.control != nr::CONTROL_CENTROID) ||
		    (agent.control != nr::CONTROL_DISTANCE)) {
				/* The objective is the area of the r-limited cell. */
				H = nr::area( agent.rlimited_cell );
			}
		break;

		case nr::PARTITIONING_ANISOTROPIC:
		case nr::PARTITIONING_ANISOTROPIC_UNCERTAINTY:
		/* The objective is the area of the cell. */
		H = nr::area( agent.cell );
		break;

		default:
		std::printf("Invalid partitioning selected.\n");
		break;
	}

	return H;
}

void nr::print(
	const nr::MA& agent,
	const int verbose,
	const int initial_spaces
) {
	/* Construct initial space string */
	std::string is;
	for (int i=0; i<initial_spaces; i++) {
		is.append(" ");
	}

	std::printf("%sMA %lu\n", is.c_str(), agent.ID);
	std::printf("%s  Position: %f %f %f\n", is.c_str(),
	    agent.position.x, agent.position.y, agent.position.z);
	/* Only print orientation if the dynamics require it. */
	if ((agent.dynamics == DYNAMICS_SI_GROUND_XYy) ||
	    (agent.dynamics == DYNAMICS_SI_AIR_XYZy) || verbose >= 3 ) {
		std::printf("%s  Attitude: %f %f %f\n", is.c_str(),
		    agent.attitude.roll, agent.attitude.pitch, agent.attitude.yaw);
	}
	/* Currently there are no dynamics that require velocities. Print only in
	   most verbose mode. */
	if (verbose >= 3) {
		std::printf("%s  Translational velocity: %f %f %f\n", is.c_str(),
		    agent.velocity_translational.x, agent.velocity_translational.y,
		    agent.velocity_translational.z);
		std::printf("%s  Rotational velocity: %f %f %f\n", is.c_str(),
		    agent.velocity_rotational.roll, agent.velocity_rotational.pitch,
		    agent.velocity_rotational.yaw);
	}
	std::printf("%s  Sensing radius: %f\n", is.c_str(), agent.sensing_radius);
	std::printf("%s  Communication radius: %f\n", is.c_str(),
	    agent.communication_radius);
	std::printf("%s  Position Uncertainty: %f\n", is.c_str(),
	    agent.position_uncertainty);
	std::printf("%s  Attitude Uncertainty: %f\n", is.c_str(),
	    agent.attitude_uncertainty);
	std::printf("%s  Relaxed Sensing Quality: %f\n", is.c_str(),
	    agent.relaxed_sensing_quality);
	std::printf("%s  Save Unassinged Sensing: %d\n", is.c_str(),
	    (int) agent.save_unassigned_sensing);
	/* Show if the various sensing polygons are empty or not. */
	std::printf("%s  Base Sensing: %d\n", is.c_str(),
	    !nr::is_empty(agent.base_sensing));
	std::printf("%s  Base Guaranteed Sensing: %d\n", is.c_str(),
	    !nr::is_empty(agent.base_guaranteed_sensing));
	std::printf("%s  Base Relaxed Sensing: %d\n", is.c_str(),
	    !nr::is_empty(agent.base_relaxed_sensing));
	std::printf("%s  Base Total Sensing: %d\n", is.c_str(),
	    !nr::is_empty(agent.base_total_sensing));
	std::printf("%s  Sensing: %d\n", is.c_str(),
	    !nr::is_empty(agent.sensing));
	std::printf("%s  Guaranteed Sensing: %d\n", is.c_str(),
	    !nr::is_empty(agent.guaranteed_sensing));
	std::printf("%s  Relaxed Sensing: %d\n", is.c_str(),
	    !nr::is_empty(agent.relaxed_sensing));
	std::printf("%s  Total Sensing: %d\n", is.c_str(),
	    !nr::is_empty(agent.total_sensing));
	std::printf("%s  Unassigned Sensing: %d\n", is.c_str(),
	    !nr::is_empty(agent.unassigned_sensing));
	/* Show if the agent's cell is empty or not. */
	std::printf("%s  Cell: %d\n", is.c_str(),
	    !nr::is_empty(agent.cell));
	std::printf("%s  r-limited cell: %d\n", is.c_str(),
	    !nr::is_empty(agent.rlimited_cell));

	/* Print all polygon vertices if verbose is greater than 1. */
	if ((verbose == 2) || (verbose == 4)) {
		std::printf("%s  Sensing: ", is.c_str());
		nr::print( agent.sensing );
		std::printf("%s  Cell: ", is.c_str());
		nr::print( agent.cell );
		std::printf("%s  R-limited cell: ", is.c_str());
		nr::print( agent.rlimited_cell );
	}

	/* Print Neighbors. */
	std::printf("%s  Neighbor IDs:", is.c_str());
	for (size_t j=0; j<agent.neighbors.size(); j++) {
		std::printf(" %lu", agent.neighbors[j].ID);
	}
	std::printf("\n");
	/* Print neighbor members if verbose is set. */
	if ((verbose == 1) || (verbose == 4)) {
		for (size_t j=0; j<agent.neighbors.size(); j++) {
			/* Indent neighbor information more than own information */
			nr::print(agent.neighbors[j], verbose, initial_spaces+4);
		}
	}
}




/****** MAs ******/
void nr::create_sensing_disks( nr::MAs* agents ) {
	/* Create the sensing disk of each agent */
	for (size_t i=0; i<agents->size(); i++) {
		nr::create_sensing_disk( &(agents->at(i)) );
	}
}

void nr::print(
	const nr::MAs& agents,
	const int verbose,
	const int initial_spaces
) {
	/* Print each agent */
	for (size_t i=0; i<agents.size(); i++) {
		nr::print( agents[i], verbose, initial_spaces );
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

double nr::calculate_objective(
    nr::MAs& agents
) {
	double H = 0;
	/* Add the objective of each agent. */
	for (size_t i=0; i<agents.size(); i++) {
		H += nr::calculate_objective( agents[i] );
	}

	/* Add the objective of the common region if it exists. */
	// if ((agents[0].partitioning == nr::PARTITIONING_ANISOTROPIC) ||
	//     (agents[0].partitioning == nr::PARTITIONING_ANISOTROPIC_UNCERTAINTY)) {
	//
	// 	nr::Polygon common;
	// 	int err;
	// 	for (size_t i=0; i<agents.size(); i++) {
	// 		err = nr::polygon_clip( nr::OR, common,
	// 		    agents[i].unassigned_sensing, &common );
	// 		if (err) {
	// 	        std::printf("Clipping operation returned error %d\n", err);
	// 	        return nr::ERROR_PARTITIONING_FAILED;
	// 	    }
	// 	}
	//
	// 	H += nr::area( common );
	// }

	if ((agents[0].partitioning == nr::PARTITIONING_ANISOTROPIC) ||
	    (agents[0].partitioning == nr::PARTITIONING_ANISOTROPIC_UNCERTAINTY)) {
		int err;
		/* Add the objective of the common region if it exists. Find the union of
		   all sensing patterns and subtract the union of all cells. */
		nr::Polygon union_sensing, union_cells, common, tmp_sensing;
		for (size_t i=0; i<agents.size(); i++) {
			/* Select correct sensing depending on the partitioning used. */
			if (agents[0].partitioning == nr::PARTITIONING_ANISOTROPIC) {
				tmp_sensing = agents[i].sensing;
			} else if (agents[i].relaxed_sensing_quality == 0) {
				tmp_sensing = agents[i].guaranteed_sensing;
			} else {
				tmp_sensing = agents[i].relaxed_sensing;
			}
			err = nr::polygon_clip( nr::OR, tmp_sensing, union_sensing, &union_sensing );
			if (err) {
				std::printf("Clipping operation returned error %d\n", err);
				return nr::ERROR_PARTITIONING_FAILED;
			}
			err = nr::polygon_clip( nr::OR, agents[i].cell, union_cells, &union_cells );
			if (err) {
				std::printf("Clipping operation returned error %d\n", err);
				return nr::ERROR_PARTITIONING_FAILED;
			}
		}

		err = nr::polygon_clip( nr::DIFF, union_sensing, union_cells, &common );
		if (err) {
			std::printf("Clipping operation returned error %d\n", err);
			return nr::ERROR_PARTITIONING_FAILED;
		}
		H += nr::area( common );
	}

	return H;
}




/************************/
/****** Simulation ******/
/************************/
int nr::export_simulation_parameters(
    struct tm* start_time,
    size_t number_of_agents,
    double simulation_duration,
    double time_step,
    double elapsed_time,
    std::vector<double>& objective,
    nr::Polygon& region,
    double objective_function_threshold,
    size_t objective_function_average_window
) {
	/*
	 *  Create filename "sim_YYYYMMDD_HHMMSS_parameters.txt".
	 *  It is 34 characters long.
	 */
	char fname[34+1];
	std::snprintf( fname, 34+1, "sim_%.4d%.2d%.2d_%.2d%.2d%.2d_parameters.txt",
	start_time->tm_year+1900, start_time->tm_mon+1, start_time->tm_mday,
	start_time->tm_hour, start_time->tm_min, start_time->tm_sec );

	/* Calculate number of iterations and average iteration time. */
	size_t number_of_iterations = std::floor(simulation_duration/time_step);
	double average_iteration = elapsed_time / number_of_iterations;

	/* Open file for writing. */
	FILE* f;
	f = std::fopen( fname, "w" );
	if (f == NULL) {
		return nr::ERROR_FILE;
	}

	/* Write data to file. */
	std::fprintf( f, "%lu\n", number_of_agents );
	std::fprintf( f, "%.*f\n", NR_FLOAT_DIGITS, simulation_duration );
	std::fprintf( f, "%.*f\n", NR_FLOAT_DIGITS, time_step );
	std::fprintf( f, "%lu\n", number_of_iterations );
	std::fprintf( f, "%.*f\n", NR_FLOAT_DIGITS, elapsed_time );
	std::fprintf( f, "%.*f\n", NR_FLOAT_DIGITS, average_iteration );
	for (size_t s=0; s<number_of_iterations; s++) {
		std::fprintf( f, "%.*f ", NR_FLOAT_DIGITS, objective[s] );
	}
	std::fprintf( f, "\n" );
	std::fprintf( f, "%.*f\n", NR_FLOAT_DIGITS, objective_function_threshold );
	std::fprintf( f, "%lu\n", objective_function_average_window );
	nr::write( region, f, true, true );

	/* Close file. */
	std::fclose( f );
	return nr::SUCCESS;
}

int nr::export_agent_parameters(
	struct tm* start_time,
    nr::MAs& agents
) {
	/* Loop over each agent. */
	for (size_t i=0; i<agents.size(); i++) {
		/*
		 *  Create filename "sim_YYYYMMDD_HHMMSS_agent_XXXX_parameters.txt".
		 *  It is 45 characters long.
		 */
		char fname[45+1];
		std::snprintf( fname, 45+1,
		"sim_%.4d%.2d%.2d_%.2d%.2d%.2d_agent_%.4lu_parameters.txt",
		start_time->tm_year+1900, start_time->tm_mon+1, start_time->tm_mday,
		start_time->tm_hour, start_time->tm_min, start_time->tm_sec,
		agents[i].ID );

		/* Open file for writing. */
		FILE* f;
		f = std::fopen( fname, "w" );
		if (f == NULL) {
			return nr::ERROR_FILE;
		}

		/* Write data to file. */
		std::fprintf( f, "%lu\n", agents[i].ID );
		std::fprintf( f, "%.*f\n", NR_FLOAT_DIGITS, agents[i].sensing_radius );
		std::fprintf( f, "%.*f\n", NR_FLOAT_DIGITS, agents[i].communication_radius );
		std::fprintf( f, "%.*f\n", NR_FLOAT_DIGITS, agents[i].position_uncertainty );
		std::fprintf( f, "%.*f\n", NR_FLOAT_DIGITS, agents[i].attitude_uncertainty );
		std::fprintf( f, "%d\n", agents[i].dynamics );
		std::fprintf( f, "%.*f\n", NR_FLOAT_DIGITS, agents[i].time_step );
		std::fprintf( f, "%d\n", agents[i].partitioning );
		std::fprintf( f, "%d\n", agents[i].control );
		std::fprintf( f, "%d\n", agents[i].avoidance );
		switch (agents[i].dynamics) {
			case DYNAMICS_SI_GROUND_XY:
			std::fprintf( f, "% .*f % .*f\n",
			NR_FLOAT_DIGITS, agents[i].control_input_gains[0],
			NR_FLOAT_DIGITS, agents[i].control_input_gains[1] );
			break;

			case DYNAMICS_SI_GROUND_XYy:
			case DYNAMICS_SI_AIR_XYZ:
			std::fprintf( f, "% .*f % .*f % .*f\n",
			NR_FLOAT_DIGITS, agents[i].control_input_gains[0],
			NR_FLOAT_DIGITS, agents[i].control_input_gains[1],
		 	NR_FLOAT_DIGITS, agents[i].control_input_gains[2] );
			break;

			case DYNAMICS_SI_AIR_XYZy:
			std::fprintf( f, "% .*f % .*f % .*f % .*f\n",
			NR_FLOAT_DIGITS, agents[i].control_input_gains[0],
			NR_FLOAT_DIGITS, agents[i].control_input_gains[1],
		 	NR_FLOAT_DIGITS, agents[i].control_input_gains[2],
		 	NR_FLOAT_DIGITS, agents[i].control_input_gains[3] );
			break;
		}
		nr::write( agents[i].base_sensing, f, true, true );
		nr::write( agents[i].base_guaranteed_sensing, f, true, true );
		nr::write( agents[i].base_relaxed_sensing, f, true, true );

		/* Close file. */
		std::fclose( f );
	}

	return nr::SUCCESS;
}




/****** NRPlot ******/
#if NR_PLOT_AVAILABLE

void nr::plot_position(
	const nr::MA& agent,
    const SDL_Color& color
) {
	/* Plot position */
	nr::plot_point( agent.position, color );
	/* Plot orientation if needed */
	if ((agent.dynamics == nr::DYNAMICS_SI_GROUND_XYy) ||
	    (agent.dynamics == nr::DYNAMICS_SI_AIR_XYZy)) {
		nr::Point v = nr::pol2cart( nr::Point( agent.sensing_radius/2, agent.attitude.yaw ) );
		nr::plot_segment( agent.position, agent.position+v, color );
	}
}

void nr::plot_positions(
	const nr::MAs& agents,
    const SDL_Color& color
) {
	/* Plot the sensing disk of each agent */
	for (size_t i=0; i<agents.size(); i++) {
		nr::plot_position( agents[i], color );
	}
}

void nr::plot_cell(
	const nr::MA& agent,
    const SDL_Color& color
) {
	nr::plot_polygon( agent.cell, color );
}

void nr::plot_cells(
	const nr::MAs& agents,
	const SDL_Color& color
) {
	for (size_t i=0; i<agents.size(); i++) {
		nr::plot_cell( agents[i], color );
	}
}

void nr::plot_sensing(
	const nr::MA& agent,
    const SDL_Color& color
) {
	nr::plot_polygon( agent.sensing, color );
}

void nr::plot_sensing(
	const nr::MAs& agents,
    const SDL_Color& color
) {
	for (size_t i=0; i<agents.size(); i++) {
		nr::plot_sensing( agents[i], color );
	}
}

void nr::plot_uncertainty(
	const nr::MA& agent,
    const SDL_Color& color
) {
	/* Positioning uncertainty */
	nr::plot_circle( nr::Circle(agent.position, agent.position_uncertainty),
	    color );
	/* Orientation uncertainty */
	if ((agent.dynamics == nr::DYNAMICS_SI_GROUND_XYy) ||
	    (agent.dynamics == nr::DYNAMICS_SI_AIR_XYZy)) {
		double vector_mangitude;
		if (agent.position_uncertainty > 0) {
			vector_mangitude = agent.position_uncertainty;
		} else {
			vector_mangitude = agent.sensing_radius/2;
		}
		nr::Point v1 = nr::pol2cart( nr::Point( vector_mangitude,
		    agent.attitude.yaw-agent.attitude_uncertainty ) );
		nr::Point v2 = nr::pol2cart( nr::Point( vector_mangitude,
		    agent.attitude.yaw+agent.attitude_uncertainty ) );
		nr::plot_segment( agent.position, agent.position+v1, color );
		nr::plot_segment( agent.position, agent.position+v2, color );
	}
}

void nr::plot_uncertainty(
	const nr::MAs& agents,
    const SDL_Color& color
) {
	for (size_t i=0; i<agents.size(); i++) {
		nr::plot_uncertainty( agents[i], color );
	}
}

void nr::plot_communication(
	const nr::MA& agent,
    const SDL_Color& color
) {
	nr::plot_circle( nr::Circle(agent.position, agent.communication_radius),
	    color );
}

void nr::plot_communication(
	const nr::MAs& agents,
    const SDL_Color& color
) {
	for (size_t i=0; i<agents.size(); i++) {
		nr::plot_communication( agents[i], color );
	}
}

#endif
