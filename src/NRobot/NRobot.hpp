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
#if NR_PLOT_AVAILABLE
#include "NRPlot.hpp"
#endif

namespace nr {

/*******************************************************/
/******************* Agent dynamics ********************/
/*******************************************************/
enum dynamics_type{
    /* Single integrator (SI) dynamics */
    DYNAMICS_SI_GROUND_XY, /* State vector: [x y] */
    DYNAMICS_SI_GROUND_XYy, /* State vector: [x y yaw] */
    DYNAMICS_SI_AIR_XYZ, /* State vector: [x y z] */
    DYNAMICS_SI_AIR_XYZy, /* State vector: [x y z yaw] */
};

/*******************************************************/
/***************** Space partitioning ******************/
/*******************************************************/
enum partitioning_type{
    PARTITIONING_VORONOI,
    PARTITIONING_GVORONOI,
    PARTITIONING_AWGVORONOI,
    PARTITIONING_ANISOTROPIC,
    PARTITIONING_ANISOTROPIC_UNCERTAINTY
};

/*******************************************************/
/******************** Control laws *********************/
/*******************************************************/
enum control_type{
    CONTROL_CENTROID,
    CONTROL_FREE_ARC,
    CONTROL_DISTANCE,
    CONTROL_ANISOTROPIC,
    CONTROL_ANISOTROPIC_UNCERTAINTY
};

/*******************************************************/
/**************** Collision avoidance ******************/
/*******************************************************/
enum avoidance_type{
    AVOIDANCE_DISK_BISECTOR
};

/*******************************************************/
/********************** MA class ***********************/
/*******************************************************/
/*!
	Mobile Agent class.
*/
class MA {
	public:
		/*********** Data members ***********/
		size_t ID;
        /* Unique agent ID. Please ensure all agents in a network have unique
           IDs. */

        /****** State ******/
		Point position;
        /* x,y,z position */
		Orientation attitude;
        /* roll,pitch,yaw attitude */
		Point velocity_translational;
        /* x,y,z velocities */
		Orientation velocity_rotational;
        /* roll,pitch,yaw velocities */

        /****** Agent parameters ******/
		double sensing_radius;
        /* The radius of the sensing pattern if it is circular, or the maximum
           distance from the agent to the boundary of its sensing patter if it
           is anisotropic. */
        double communication_radius;
        /* The radius inside which the MA can communicate with other MAs. */
		double position_uncertainty;
        /* The maximum distance the MAs real position may be from its reported
           position. */
        double attitude_uncertainty;
        double relaxed_sensing_quality;
        /* The coverage quality at the relaxed sensing region. */
        bool save_unassigned_sensing;
        /* Whether to save the part of the MA's sensing region left unassigned.
           Applies to certain partitioning schemes only.  */

        /****** Sensing and cell ******/
        Polygon base_sensing;
        /* The sensing pattern of the MA when located at [x,y]=[0,0] with
           theta=0. */
        Polygon base_guaranteed_sensing;
        /* The region the MA is guaranteed to sense given its uncertainty when
           the MA is located at [x,y]=[0,0] with theta=0. */
        Polygon base_relaxed_sensing;
        /* The region the MA is not guaranteed to sense excluding the region it
           is guaranteed not to sense when the MA is located at [x,y]=[0,0]
           with theta=0. */
        Polygon base_total_sensing;
        /* The union of the guaranteed and relaxed sensing regions when the MA
           is located at [x,y]=[0,0] with theta=0. */
		Polygon sensing;
        /* The curent sensing pattern of the agent. */
        Polygon guaranteed_sensing;
        /* The region the MA is guaranteed to sense given its uncertainty */
        Polygon relaxed_sensing;
        /* The region the MA is not guaranteed to sense excluding the region it
           is guaranteed not to sense. */
        Polygon total_sensing;
        /* The union of the guaranteed and relaxed sensing regions. */
        Polygon unassigned_sensing;
        /* The part of the MA's sensing region left unassigned during the
        partitioning. Applies to certain partitioning schemes only. */
		Polygon cell;
        /* The region assigned to the MA. */
		Polygon rlimited_cell;
        /* The interesection of the MA's cell with its circular sensing
           pattern. */

        /****** Neighbors ******/
		std::vector<MA> neighbors;
        /* The agents inside the MA's communication radius. */

        /****** Control ******/
		partitioning_type partitioning;
        /* The partitioning scheme (if any) used in conjuction with the control
           law. */
		control_type control;
        /* The type of the control law used. Tightly coupled with the
           dynamics. */
       avoidance_type avoidance;
       /* The type of collision avoidance used. This is used after the control
          input has been computed. */
        std::vector<double> control_input;
        /* The control input vector. Its length and elements depend on the
           chosen dynamics. The default constructors create a vector of 6
           elements. */
        std::vector<double> control_input_gains;
        /* The control input gains. Its length and elements depend on the
          chosen dynamics. The default constructors create a vector of 6
          elements with value 1. */

        /****** Dynamics and simulation ******/
        dynamics_type dynamics;
        /* The type of the dynamics used, see enum dynamics_type. */
        double time_step;
        /* The time step when simulating the MA dynamics. */

		/*********** Constructors ***********/
		MA();

		MA(
			Point& pos,
            double time_step = 0.01,
			double sradius = 0,
			double uradius = 0,
			double cradius = 0
		);

		MA(
			Point& pos,
			Orientation& att,
            double time_step = 0.01,
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
		/*********** Data members ***********/

		/*********** Constructors ***********/
		MAs();

        MAs(
			Points& pos,
            double time_step = 0.01
		);

		MAs(
			Points& pos,
            double time_step,
			std::vector<double>& sradii,
			std::vector<double>& uradii,
			std::vector<double>& cradii
		);

        MAs(
			Points& pos,
			Orientations& att,
            double time_step = 0.01
		);

		MAs(
			Points& pos,
			Orientations& att,
            double time_step,
			std::vector<double>& sradii,
			std::vector<double>& uradii,
			std::vector<double>& cradii
		);
};





/**********************************************************/
/********************* Main functions *********************/
/**********************************************************/
void info();



/*****************/
/****** MA ******/
/*****************/
void create_sensing_disk(
    MA* agent
);
/* Set the MAs sensing pattern to a disk with radius equal to sensing_radius. */

void find_neighbors(
    MA* agent,
    const MAs& agents
);
/* Find the neighbors of the MA and store them in the neighbors vector. */

void simulate_dynamics(
    MA* agent
);
/* Use the control_input vector to simulate the MA dynamics and update its
   state. */

int compute_base_sensing_patterns(
    MA* agent
);
/* Compute the base guaranteed, relaxed and total sensing patterns. This is
   used during the initialization phase in order to speed up execution by only
   translating and rotating the base sensing patterns when MA state changes. */

void update_sensing_patterns(
    MA* agent
);
/* Rotate and translate the base sensing patterns according to the MAs state. */

int compute_cell(
    MA* agent,
    const Polygon& region
);
/* Compute the assigned cell of the MA based on the value of the partitioning
   variable. */

void compute_control(
    MA* agent
);
/* Compute the value of the control input for the MA based on the value of the
   control variable. */

void ensure_collision_avoidance(
    MA* agent
);
/* Modifies the control input so that the agent will not collide with any of
   its neighbors. */

double calculate_objective(
   MA& agent
);
/* Computes the value of the objective function for the current agent. The
   computation depends on the control law and partitioning selected. */

void print(
    const MA& agent,
    const int verbose = 0,
    const int initial_spaces = 0
);
/* Print information about the MA on stdout, depending on the dynamics,
   partitioning and control law of the current MA. If verbose = 1, then the
   same info about the neighbors will be printed as well. If verbose = 2, then
   all polygon vertices of the current agent will be printed. If verbose = 3,
   then all information about the current agent will be printed, regardless
   of dynamics, partitioning or control law, excluding polygon vertices. If
   verbose = 4, then all information, including polygon vertices will be
   printed for the current agent and all of its neighbors. By setting
   initial_spaces, the number of spaces added before each line can be set. This
   is also used internally by print when printing neighbors. */





/*****************/
/****** MAs ******/
/*****************/
void create_sensing_disks(
    MAs* agents
);

void print(
    const MAs& agents,
    const int verbose = 0,
    const int initial_spaces = 0
);

void set_partitioning(
    MAs* agents,
    const partitioning_type partitioning
);

void set_control(
    MAs* agents,
    const control_type control
);

double calculate_objective(
    MAs& agents
);
/* Computes the value of the objective function for the current configuration.
   The computation depends on the control law and partitioning selected. */





/************************/
/****** Simulation ******/
/************************/
int export_results(
    Polygon& region,
    MAs& agents,
    std::vector<double> objective,
    double duration,
    double time_step,
    double elapsed_time
);
/* Export the simulation results to three text files. */



/********************/
/****** NRPlot ******/
/********************/
#if NR_PLOT_AVAILABLE

void plot_position(
    const MA& agent,
    const SDL_Color& color = PLOT_FOREGROUND_COLOR
);

void plot_positions(
    const MAs& agents,
    const SDL_Color& color = PLOT_FOREGROUND_COLOR
);
/* Plot the position and orientation of a single MA or a vector of MAs. */

void plot_cell(
    const MA& agent,
    const SDL_Color& color = PLOT_FOREGROUND_COLOR
);

void plot_cells(
    const MAs& agents,
    const SDL_Color& color = PLOT_FOREGROUND_COLOR
);
/* Plot the cell of a single MA or a vector of MAs. */

void plot_sensing(
    const MA& agent,
    const SDL_Color& color = PLOT_FOREGROUND_COLOR
);

void plot_sensing(
    const MAs& agents,
    const SDL_Color& color = PLOT_FOREGROUND_COLOR
);
/* Plot the sensing pattern of a single MA or a vector of MAs. */

void plot_uncertainty(
    const MA& agent,
    const SDL_Color& color = PLOT_FOREGROUND_COLOR
);

void plot_uncertainty(
    const MAs& agents,
    const SDL_Color& color = PLOT_FOREGROUND_COLOR
);
/* Plot the positioning and orientation uncertainty of a single MA or a vector
   of MAs. */

void plot_communication(
    const MA& agent,
    const SDL_Color& color = PLOT_FOREGROUND_COLOR
);

void plot_communication(
    const MAs& agents,
    const SDL_Color& color = PLOT_FOREGROUND_COLOR
);
/* Plot the communication radius of a single MA or a vector of MAs. */

#endif

} /* End of namespace */

#endif
