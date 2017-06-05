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

#ifndef __NRobot_hpp
#define __NRobot_hpp

#include "NRBase.hpp"

namespace nr {

/*******************************************************/
/***************** Space partitioning ******************/
/*******************************************************/
enum partitioning_type{
    PARTITIONING_VORONOI,
    PARTITIONING_GVORONOI,
    PARTITIONING_AWGVORONOI
};

/*******************************************************/
/******************** Control laws *********************/
/*******************************************************/
enum control_type{
    CONTROL_CENTROID,
    CONTROL_FREE_ARC
};

/*******************************************************/
/********************** MA class ***********************/
/*******************************************************/
/*!
	Mobile Agent class.
*/
class MA {
	public:
		/****** Data members ******/
		size_t ID;
		Point position;
		Orientation attitude;
		Point velocity_translational;
		Orientation velocity_rotational;
		double sensing_radius;
		double uncertainty_radius;
		double communication_radius;
		Polygon sensing;
		Polygon cell;
		Polygon rlimited_cell;
		std::vector<MA> neighbors;
		partitioning_type partitioning;
		control_type control;

		/****** Constructors ******/
		MA();
		MA(
			Point& pos,
			double sradius = 0,
			double uradius = 0,
			double cradius = 0
		);
		MA(
			Point& pos,
			Orientation& att,
			double sradius = 0,
			double uradius = 0,
			double cradius = 0
		);
};




/*******************************************************/
/********************** MAs class **********************/
/*******************************************************/
/*!
	Vector of Mobile Agents.
*/
class MAs: public std::vector<MA> {
	public:
		/****** Data members ******/

		/****** Constructors ******/
		MAs();
		MAs(
			Points& pos,
			std::vector<double>& sradii,
			std::vector<double>& uradii,
			std::vector<double>& cradii
		);
		MAs(
			Points& pos,
			Orientations& att,
			std::vector<double>& sradii,
			std::vector<double>& uradii,
			std::vector<double>& cradii
		);
};





/**********************************************************/
/********************* Main functions *********************/
/**********************************************************/
void info();
/****** MA ******/
void create_sensing_disk( MA* agent );
void find_neighbors( MA* agent, const MAs& agents );
int compute_cell( MA* agent, const Polygon& region );
void compute_control( MA* agent );
void print( const MA& agent, const bool verbose = false );
void plot_position( const MA& agent );
void plot_cell( const MA& agent );
void plot_sensing( const MA& agent );
void plot_uncertainty( const MA& agent );
void plot_communication( const MA& agent );

/****** MAs ******/
void create_sensing_disks( MAs* agents );
void print( const MAs& agents, const bool verbose = false );
void set_partitioning( MAs* agents, const partitioning_type partitioning );
void set_control( MAs* agents, const control_type control );
void plot_positions( const MAs& agents );
void plot_cells( const MAs& agents );
void plot_sensing( const MAs& agents );
void plot_uncertainty( const MAs& agents );
void plot_communication( const MAs& agents );

} /* End of namespace */

#endif
