/*
	Copyright (C) 2017 Sotiris Papatheodorou

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

#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>

#include "NR.hpp"
#include "Jn.hpp"
#include "n.hpp"
#include "J.hpp"

int main() {
    nr::info();

    /* Setup agents */
    double c = 2;
    double xi = -c;
    double yi = 0;
    double ri = 0.1;
    double Ri = 0.8;
    double xj = c;
    double yj = 0;
    double rj = 0.1;
    double Rj = 0.8;
    double ai = ri + rj + Rj - Ri;

    /* Create parameter t vector */
    size_t Nt = 101;
    std::vector<double> t (Nt, 0);
    double tmax = std::ceil( std::acosh(15/std::abs(ai)) );
    double dt = 2*tmax/(Nt-1);
    for (size_t i=0; i<Nt; i++) {
        t[i] = -tmax + dt*i;
    }

    /* Compute Jacobians */
    std::vector<double> Ji_x_xi (Nt, 0), Ji_x_yi (Nt, 0), Ji_y_xi (Nt, 0), Ji_y_yi (Nt, 0);
    std::vector<double> Jj_x_xi (Nt, 0), Jj_x_yi (Nt, 0), Jj_y_xi (Nt, 0), Jj_y_yi (Nt, 0);
    for (size_t i=0; i<Nt; i++) {
        Ji_x_xi[i] = nr_FJi_x_xi( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
        Ji_x_yi[i] = nr_FJi_x_yi( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
        Ji_y_xi[i] = nr_FJi_y_xi( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
        Ji_y_yi[i] = nr_FJi_y_yi( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );

        Jj_x_xi[i] = nr_FJj_x_xi( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
        Jj_x_yi[i] = nr_FJj_x_yi( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
        Jj_y_xi[i] = nr_FJj_y_xi( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
        Jj_y_yi[i] = nr_FJj_y_yi( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
    }

    /* Compute normal vectors */
    std::vector<double> ni_x (Nt, 0), ni_y (Nt, 0), nj_x (Nt, 0), nj_y (Nt, 0);
    for (size_t i=0; i<Nt; i++) {
        ni_x[i] = nr_Fni_x( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
        ni_y[i] = nr_Fni_y( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
        nj_x[i] = nr_Fnj_x( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
        nj_y[i] = nr_Fnj_y( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
    }

    /* Compute Jacobian-normal products */
    std::vector<double> Jni_x (Nt, 0), Jni_y (Nt, 0), Jnj_x (Nt, 0), Jnj_y (Nt, 0);
    for (size_t i=0; i<Nt; i++) {
        Jni_x[i] = nr_FJni_x( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
        Jni_y[i] = nr_FJni_y( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
        Jnj_x[i] = nr_FJnj_x( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
        Jnj_y[i] = nr_FJnj_y( t[i], xi, yi, ri, Ri, xj, yj, rj, Rj );
    }

    /* Export vectors to file */
    FILE* fp;
    fp = std::fopen("bin/J_values.txt", "w");
    if (fp != NULL) {
        for (size_t i=0; i<Nt; i++) {
            std::fprintf(fp, "% lf % lf % lf % lf % lf % lf % lf % lf % lf\n", t[i],
                Ji_x_xi[i], Ji_x_yi[i], Ji_y_xi[i], Ji_y_yi[i],
                Jj_x_xi[i], Jj_x_yi[i], Jj_y_xi[i], Jj_y_yi[i]);
        }
    }
    std::fclose(fp);

    fp = std::fopen("bin/n_values.txt", "w");
    if (fp != NULL) {
        for (size_t i=0; i<Nt; i++) {
            std::fprintf(fp, "% lf % lf % lf % lf % lf\n", t[i],
                ni_x[i], ni_y[i], nj_x[i], nj_y[i]);
        }
    }
    std::fclose(fp);

    fp = std::fopen("bin/Jn_values.txt", "w");
    if (fp != NULL) {
        for (size_t i=0; i<Nt; i++) {
            std::fprintf(fp, "% lf % lf % lf % lf % lf\n", t[i],
                Jni_x[i], Jni_y[i], Jnj_x[i], Jnj_y[i]);
        }
    }
    std::fclose(fp);

    std::printf("Results written to files\n");

    /* Calculate AWGV cells */
    nr::Polygon region;
	nr::read( &region, "resources/region_sq.txt", true);
    size_t N = 2;
    nr::Points P;
	P.push_back( nr::Point(xi,yi) );
	P.push_back( nr::Point(xj,yj) );
    std::vector<double> uradii { ri, rj };
	std::vector<double> sradii { Ri, Rj };
    nr::Circles udisks;
	for (size_t i=0; i<N; i++) {
		udisks.push_back( nr::Circle(P[i], uradii[i]) );
	}
    nr::Polygons AWGV;
	AWGV.resize(udisks.size());
	for (size_t i=0; i<udisks.size(); i++) {
		nr::awg_voronoi_cell( region, udisks, sradii, i, &(AWGV[i]) );
	}

    /* Plot */
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
            nr::plot_point( nr::Point(xi,yi), PLOT_FOREGROUND_COLOR, 2 );
            nr::plot_circle( nr::Circle( nr::Point(xi,yi), ri ) );
            nr::plot_circle( nr::Circle( nr::Point(xi,yi), Ri ) );
            nr::plot_polygon( AWGV[0] );
            /* Plot normal vectors */
            for (size_t i=0; i<AWGV[0].contour[0].size(); i++) {
                nr::plot_segment( AWGV[0].contour[0][i], AWGV[0].contour[0][i] + nr::Point(ni_x[i], ni_y[i]) );
            }

            /* Red for agent j */
			PLOT_FOREGROUND_COLOR = {0xAA, 0x00, 0x00, 0xFF};
            nr::plot_point( nr::Point(xj,yj), PLOT_FOREGROUND_COLOR, 2 );
            nr::plot_circle( nr::Circle( nr::Point(xj,yj), rj ) );
            nr::plot_circle( nr::Circle( nr::Point(xj,yj), Rj ) );
            nr::plot_polygon( AWGV[1] );
            /* Plot normal vectors */
            for (size_t i=0; i<AWGV[1].contour[0].size(); i++) {
                nr::plot_segment( AWGV[1].contour[0][i], AWGV[1].contour[0][i] + nr::Point(nj_x[i], nj_y[i]) );
            }


			nr::plot_render();
			uquit = nr::plot_handle_input();
		}
		nr::plot_quit();
	#endif

    return 0;
}
