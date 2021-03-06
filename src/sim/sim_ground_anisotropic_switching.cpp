/*
 *  Copyright (C) 2017 Sotiris Papatheodorou
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

#include <cstdio>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include <numeric>

#include <NR.hpp>

double average( std::vector<double>& v, size_t begin, size_t end ) {
    size_t N = end - begin + 1;
    double sum = 0;

    for (size_t i=begin; i<begin+N; i++) {
        sum += v[i];
    }

    return sum/N;
}

int main() {
    /****** Simulation parameters ******/
	double Tfinal = 5;
	double Tstep = 0.01;
    size_t plot_sleep_ms = 10;
	bool export_results = false;

	/* Get the current time. */
	clock_t start_time_raw = std::time(NULL);
	struct tm* start_time = std::localtime( &start_time_raw );
	/* Number of iterations */
	size_t smax = std::floor(Tfinal/Tstep);

	nr::info();

	/****** Region of interest ******/
	nr::Polygon region;
	nr::read( &region, "resources/region_cb.txt", true);

	/****** Setup agents ******/
    /* Agent initial positions */
	nr::Points P;
    P.push_back( nr::Point(1.721,1.013) );
	P.push_back( nr::Point(1.482,1.206) );
	P.push_back( nr::Point(2.006,1.342) );
	P.push_back( nr::Point(1.536,1.454) );
	P.push_back( nr::Point(1.443,1.655) );
	P.push_back( nr::Point(1.792,1.585) );
	P.push_back( nr::Point(1.255,1.134) );
	P.push_back( nr::Point(1.911,0.905) );
	/* Agent initial attitudes */
	nr::Orientations A;
    A.push_back( nr::Orientation(0,0, 2.4679773854259808 ) );
	A.push_back( nr::Orientation(0,0, 0.28861356578484565 ) );
	A.push_back( nr::Orientation(0,0, 4.9641841747027469 ) );
	A.push_back( nr::Orientation(0,0, 0.274211804968107 ) );
	A.push_back( nr::Orientation(0,0, 5.672512046080453 ) );
	A.push_back( nr::Orientation(0,0, 1.3573179379420355 ) );
	A.push_back( nr::Orientation(0,0, 0.5407470134652721 ) );
	A.push_back( nr::Orientation(0,0, 1.4436339452103413 ) );
	/* Number of agents */
	size_t N = P.size();
    /* Base sensing */
	nr::Polygon base_sensing = nr::Polygon( nr::Ellipse( 0.5, 0.3, nr::Point(0.25,0) ) );
	/* Uncertainty */
	std::vector<double> position_uncertainty (N,0.1);
	std::vector<double> attitude_uncertainty (N,M_PI/10);
	std::vector<double> communication_radius (N,2*nr::radius( base_sensing ));
	/* Control input gains */
	std::vector<double> control_input_gains = {1,1,10};
	/* Initialize agents */
	nr::MAs agents (
		nr::DYNAMICS_SI_GROUND_XYy,
		nr::PARTITIONING_ANISOTROPIC_UNCERTAINTY,
		nr::CONTROL_ANISOTROPIC_UNCERTAINTY,
		P,
		A,
		position_uncertainty,
		attitude_uncertainty,
		communication_radius,
		base_sensing,
		control_input_gains,
		Tstep
	);
	for (size_t i=0; i<N; i++) {
        /* Change communication radius */
        agents[i].communication_radius = 2 * (agents[i].sensing_radius + agents[i].position_uncertainty);
		/* Sensing quality at feasible sensing */
		agents[i].feasible_sensing_quality = 1;
	}

    /****** Create constrained regions ******/
    nr::Polygons offset_regions;
    for (size_t i=0; i<N; i++) {
        offset_regions.push_back( region );

        int err = nr::offset_in( &(offset_regions[i]), agents[i].position_uncertainty );
        if (err) {
            return nr::ERROR_CLIPPING_FAILED;
        }
    }


    /****** Initialize MA evolution vector ******/
	std::vector<nr::MA_evolution> agents_evolution (N, nr::MA_evolution());
	if (export_results) {
		for (size_t i=0; i<N; i++) {
			agents_evolution[i] =
			nr::MA_evolution( agents[i].ID, N, smax, agents[i].dynamics );
		}
	}


	/****** Initialize plot ******/
	#if NR_PLOT_AVAILABLE
	if (nr::plot_init()) exit(1);
    PLOT_SCALE = 200;
	PLOT_X_OFFSET = -300;
	PLOT_Y_OFFSET = 200;
	bool uquit = false;
	#endif



	/****** Simulate agents ******/
	std::vector<double> H (smax, 0);
    std::vector<std::vector<double>> Hi (N, std::vector<double> (smax, 0));
    double initial_feasible_sensing_quality = agents[0].feasible_sensing_quality;
    std::vector<bool> converged (N, false);
    double H_threshold = 0.001;
    double window_time = 0.5;
    size_t window_size = std::floor(window_time / Tstep);
	#if NR_TIME_EXECUTION
	clock_t begin, end;
	begin = std::clock();
	#endif

	for (size_t s=1; s<=smax; s++) {

		/* Each agent computes its own control input separately */
		for (size_t i=0; i<N; i++) {
            /* Translate and rotate for real sensing */
    		nr::update_sensing_patterns( &(agents[i]) );
		}
		for (size_t i=0; i<N; i++) {
			/* Communicate with neighbors and get their states */
			nr::find_neighbors( &(agents[i]), agents );
            /* Check if switching of the control law is required. */
            /* If all neighbors have converged too, change the feasible sensing
               quality. */
            /* Initialize to the agents own convergence status. */
            bool neighbors_converged = converged[agents[i].ID-1];
            for (size_t j=0; j<agents[i].neighbors.size(); j++) {
                if (!converged[agents[i].neighbors[j].ID-1]) {
                    neighbors_converged = false;
                    break;
                }
            }
            if (neighbors_converged && agents[i].feasible_sensing_quality == initial_feasible_sensing_quality) {
                agents[i].feasible_sensing_quality = !agents[i].feasible_sensing_quality;
                std::printf("Agent %lu switched at iteration %lu\n", agents[i].ID, s);
            }
			/* Compute own cell using neighbors vector. */
			nr::compute_cell( &(agents[i]), region );
			/* Compute own control input. */
			nr::compute_control( &(agents[i]) );
            /* Ensure collision avoidance. */
            nr::ensure_collision_avoidance( &(agents[i]) );
		}

		/* Calculate objective function and print progress. */
		H[s-1] = nr::calculate_objective( agents );
        for (size_t i=0; i<N; i++) {
            Hi[i][s-1] = nr::calculate_objective( agents[i] );
            /* Compare value with rolling average. If difference smaller than
               threshold, mark the agent as converged. */
            if (s > window_size) {
                double rolling_average = average( Hi[i], s-2-(window_size-1), s-2 );
                double diff = std::abs(rolling_average-Hi[i][s-1]);
                if ( diff < H_threshold &&
                    (agents[i].feasible_sensing_quality == initial_feasible_sensing_quality) &&
                    !converged[agents[i].ID-1]) {
                    /* H increase is below the threshold and the agent hasn't
                       converged. */
                    converged[agents[i].ID-1] = true;
                    std::printf("Agent %lu converged at iteration %lu\n", agents[i].ID, s);
                }
            }
        }
		std::printf("Iteration: %lu    H: %.4f\r", s, H[s-1]);

        /* Save agent evolution. */
		if (export_results) {
            for (size_t i=0; i<N; i++) {
    			agents_evolution[i].position.push_back(agents[i].position);
    			agents_evolution[i].attitude.push_back(agents[i].attitude);
    			agents_evolution[i].velocity_translational.
    			  push_back(agents[i].velocity_translational);
    			agents_evolution[i].velocity_rotational.
    			  push_back(agents[i].velocity_rotational);
    			agents_evolution[i].feasible_sensing_quality.
    			  push_back(agents[i].feasible_sensing_quality);
    			for (size_t j=0; j<agents[i].neighbors.size(); j++) {
    				agents_evolution[i].
    				  neighbor_connectivity[agents[i].neighbors[j].ID-1].
    				  push_back(true);
    			}
    			for (size_t j=0; j<agents_evolution[i].control_input.size(); j++) {
    				agents_evolution[i].control_input[j].push_back
    				  (agents[i].control_input[j]);
    			}
    		}
		}

		// nr::print( agents, false );

		/* Plot network state */
		#if NR_PLOT_AVAILABLE
		nr::plot_clear_render();
		nr::plot_show_axes();

		/* Region, nodes and udisks */
		nr::plot_polygon( region, BLACK );
		nr::plot_positions( agents, BLACK );
		nr::plot_uncertainty( agents, BLACK );
		/* sdisks */
		// nr::plot_sensing( agents, RED );
		/* cells */
        // nr::plot_cells( agents, BLUE );
		for (size_t i=0; i<N; i++) {
			nr::plot_cell( agents[i], PLOT_COLORS[i % PLOT_COLORS.size()] );
		}
		/* communication */
		// nr::plot_communication_links( agents, GREEN );

		nr::plot_render();
		uquit = nr::plot_handle_input();
		if (uquit) {
			break;
		}
        std::this_thread::sleep_for(std::chrono::milliseconds(plot_sleep_ms));
		#endif

		/* The movement of each agent is simulated */
		for (size_t i=0; i<N; i++) {
			nr::simulate_dynamics( &(agents[i]) );
		}
	}




	/****** Print simulation info ******/
	#if NR_TIME_EXECUTION
	end = clock();
	double elapsed_time = (double)(end - begin) / CLOCKS_PER_SEC;
	double average_iteration = elapsed_time / smax;
	std::printf("Simulation finished in %.2f seconds\n", elapsed_time);
	std::printf("Average iteration %.5f seconds\n", average_iteration);
	#endif

    /****** Export simulation results ******/
	if (export_results) {
		int err;
		err = nr::export_simulation_parameters( start_time, N, Tfinal, Tstep,
		elapsed_time, H, region, H_threshold, window_size );
		if (err) {
			return nr::ERROR_FILE;
		}
		err = nr::export_agent_parameters( start_time, agents );
		if (err) {
			return nr::ERROR_FILE;
		}
		err = nr::export_agent_state( start_time, agents_evolution );
		if (err) {
			return nr::ERROR_FILE;
		}
	}

	/****** Quit plot ******/
    #if NR_PLOT_AVAILABLE
	uquit = false;
	while (!uquit) {
		nr::plot_clear_render();
		nr::plot_show_axes();

		/* Region, nodes and udisks */
		nr::plot_polygon( region, BLACK );
		nr::plot_positions( agents, BLACK );
		nr::plot_uncertainty( agents, BLACK );
		/* sdisks */
		// nr::plot_sensing( agents, RED );
		/* cells */
		// nr::plot_cells( agents, BLUE );
		for (size_t i=0; i<N; i++) {
			nr::plot_cell( agents[i], PLOT_COLORS[i % PLOT_COLORS.size()] );
		}
		/* communication */
		// nr::plot_communication( agents, GREEN );

		nr::plot_render();
		uquit = nr::plot_handle_input();
		if (uquit) {
			break;
		}
	}
	nr::plot_quit();
	#endif

	return 0;
}
