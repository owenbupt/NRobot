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

#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>

#include <NR.hpp>
#include "Jn.hpp"
#include "n.hpp"
#include "J.hpp"

int main() {
    nr::info();

    /* Options */
    #if NR_PLOT_AVAILABLE
        bool PLOT_NORMALS = true;
        bool PLOT_NEW_CELL = false;
    #endif

    /* Setup agents */
    double c = 2;
    nr::Points q;
    q.push_back( nr::Point(-c,0) );
    q.push_back( nr::Point(c,0) );
    size_t N = q.size();
    std::vector<double> r { 0.5, 0.5 };
	std::vector<double> R { 1.5, 0.1 };
    std::vector<double> Rg;
    for (size_t k=0; k<N; k++) {
        Rg.push_back( R[k]-r[k] );
    }

    /* Calculate AWGV cells */
    nr::Polygon region;
	nr::read( &region, "resources/region_sq.txt", true);
    nr::scale( &region, 5 );
    nr::Circles udisks;
	for (size_t i=0; i<N; i++) {
		udisks.push_back( nr::Circle(q[i], r[i]) );
	}
    nr::Polygons AWGV;
	AWGV.resize(udisks.size());
	for (size_t i=0; i<udisks.size(); i++) {
		nr::awg_voronoi_cell( region, udisks, R, i, &(AWGV[i]) );
	}

    /* Select i and j agents */
    size_t i = 0;
    size_t j = 1;

    /* Agent i */
    size_t Nv = AWGV[i].contour[0].size();
    std::vector<double> ti (Nv, 0);
    std::vector<double> ni_x (Nv, 0), ni_y (Nv, 0);
    std::vector<double> Ji_x_xi (Nv, 0), Ji_x_yi (Nv, 0), Ji_y_xi (Nv, 0), Ji_y_yi (Nv, 0);
    std::vector<double> Jni_x (Nv, 0), Jni_y (Nv, 0);
    for (size_t k=0; k<AWGV[i].contour[0].size(); k++) {
        /* Get the current cell vertex */
        nr::Point v = AWGV[i].contour[0][k];
        /* Translate */
        v = v - nr::midpoint( q[i], q[j] );
        /* Rotate */
        double theta = std::atan2(q[j].y-q[i].y, q[j].x-q[i].x);;
        v = nr::rotate( v, -theta );
        /* Find the t parameter value */
        double ai = (r[i] + r[j] + R[j] - R[i])/2;
        double c = nr::norm( q[i]-q[j] )/2;
        double bi = std::sqrt( c*c - ai*ai );
        ti[k] = -std::asinh(v.y/bi); /* WHY IS MINUS NEEDED HERE? */

        /* Normal */
        ni_x[k] = nr_Fni_x( ti[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        ni_y[k] = nr_Fni_y( ti[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        /* Jacobian */
        Ji_x_xi[i] = nr_FJi_x_xi( ti[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        Ji_x_yi[i] = nr_FJi_x_yi( ti[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        Ji_y_xi[i] = nr_FJi_y_xi( ti[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        Ji_y_yi[i] = nr_FJi_y_yi( ti[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        /* Jacobian-Normal product */
        Jni_x[i] = nr_FJni_x( ti[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        Jni_y[i] = nr_FJni_y( ti[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
    }


    /* Agent j */
    Nv = AWGV[j].contour[0].size();
    std::vector<double> tj (Nv, 0);
    std::vector<double> nj_x (Nv, 0), nj_y (Nv, 0);
    std::vector<double> Jj_x_xi (Nv, 0), Jj_x_yi (Nv, 0), Jj_y_xi (Nv, 0), Jj_y_yi (Nv, 0);
    std::vector<double> Jnj_x (Nv, 0), Jnj_y (Nv, 0);
    for (size_t k=0; k<AWGV[j].contour[0].size(); k++) {
        /* Get the current cell vertex */
        nr::Point v = AWGV[j].contour[0][k];
        /* Translate */
        v = v - nr::midpoint( q[j], q[i] );
        /* Rotate */
        double theta = std::atan2(q[i].y-q[j].y, q[i].x-q[j].x);;
        v = nr::rotate( v, -theta );
        /* Find the t parameter value */
        double aj = (r[j] + r[i] + R[i] - R[j])/2;
        double c = nr::norm( q[j]-q[i] )/2;
        double bj = std::sqrt( c*c - aj*aj );
        tj[k] = -std::asinh(v.y/bj); /* WHY IS MINUS NEEDED HERE? */

        /* Normal */
        nj_x[k] = nr_Fnj_x( tj[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        nj_y[k] = nr_Fnj_y( tj[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        /* Jacobian */
        Jj_x_xi[i] = nr_FJj_x_xi( tj[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        Jj_x_yi[i] = nr_FJj_x_yi( tj[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        Jj_y_xi[i] = nr_FJj_y_xi( tj[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        Jj_y_yi[i] = nr_FJj_y_yi( tj[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        /* Jacobian-Normal product */
        Jnj_x[i] = nr_FJnj_x( tj[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
        Jnj_y[i] = nr_FJnj_y( tj[k], q[i].x, q[i].y, r[i], R[i], q[j].x, q[j].y, r[j], R[j] );
    }


    /************************* Export to files ********************************/
    FILE* fp;
    fp = std::fopen("bin/J_values.txt", "w");
    if (fp != NULL) {
        for (size_t k=0; k<Nv; k++) {
            std::fprintf(fp, "% lf % lf % lf % lf % lf % lf % lf % lf % lf\n", ti[k],
                Ji_x_xi[k], Ji_x_yi[k], Ji_y_xi[k], Ji_y_yi[k],
                Jj_x_xi[k], Jj_x_yi[k], Jj_y_xi[k], Jj_y_yi[k]);
        }
    }
    std::fclose(fp);

    fp = std::fopen("bin/n_values.txt", "w");
    if (fp != NULL) {
        for (size_t k=0; k<Nv; k++) {
            std::fprintf(fp, "% lf % lf % lf % lf % lf\n", ti[k],
                ni_x[k], ni_y[k], nj_x[k], nj_y[k]);
        }
    }
    std::fclose(fp);

    fp = std::fopen("bin/Jn_values.txt", "w");
    if (fp != NULL) {
        for (size_t k=0; k<Nv; k++) {
            std::fprintf(fp, "% lf % lf % lf % lf % lf\n", ti[k],
                Jni_x[k], Jni_y[k], Jnj_x[k], Jnj_y[k]);
        }
    }
    std::fclose(fp);

    std::printf("Results written to files\n");

    /************************* Change in cell of i ****************************/
    nr::Point dq (0.5, 0);
    udisks[i].center += dq;
    nr::Polygon AWGVi_new;
    nr::awg_voronoi_cell( region, udisks, R, i, &AWGVi_new );
    /* Translate to show relative change */
    // nr::translate( &AWGVi_new, -dq );
    udisks[i].center -= dq;

    /******************************* Plot *************************************/
	#if NR_PLOT_AVAILABLE
		if (nr::plot_init()) exit(1);
        PLOT_BACKGROUND_COLOR = {0x30, 0x30, 0x30, 0xFF};
		PLOT_SCALE = 50;
		bool uquit = false;

		while (!uquit) {
			nr::plot_clear_render();
			nr::plot_show_axes();

            /* Black for region */
			PLOT_FOREGROUND_COLOR = {0x00, 0x00, 0x00, 0xFF};
			nr::plot_polygon( region );

            /* Blue for agent i */
			PLOT_FOREGROUND_COLOR = {0x00, 0x00, 0xAA, 0xFF};
            nr::plot_point( q[i], PLOT_FOREGROUND_COLOR, 2 );
            nr::plot_circle( nr::Circle( q[i], r[i] ) );
            nr::plot_circle( nr::Circle( q[i], R[i] ) );
            nr::plot_polygon( AWGV[i] );
            /* Plot normal vectors */
            if (PLOT_NORMALS) {
                for (size_t k=0; k<AWGV[i].contour[0].size(); k++) {
                    nr::plot_segment( AWGV[i].contour[0][k], AWGV[i].contour[0][k] + nr::Point(ni_x[k], ni_y[k]) );
                }
            }
            if (PLOT_NEW_CELL) {
                PLOT_FOREGROUND_COLOR = {0x00, 0xAA, 0x00, 0xFF};
                nr::plot_polygon( AWGVi_new );
            }

            /* Red for agent j */
			PLOT_FOREGROUND_COLOR = {0xAA, 0x00, 0x00, 0xFF};
            nr::plot_point( q[j], PLOT_FOREGROUND_COLOR, 2 );
            nr::plot_circle( nr::Circle( q[j], r[j] ) );
            nr::plot_circle( nr::Circle( q[j], R[j] ) );
            nr::plot_polygon( AWGV[j] );
            /* Plot normal vectors */
            if (PLOT_NORMALS) {
                for (size_t k=0; k<AWGV[j].contour[0].size(); k++) {
                    nr::plot_segment( AWGV[j].contour[0][k], AWGV[j].contour[0][k] + nr::Point(nj_x[k], nj_y[k]) );
                }
            }


			nr::plot_render();
			uquit = nr::plot_handle_input();
		}
		nr::plot_quit();
	#endif

    return 0;
}
