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

int main() {
    /* Setup agents */
    double c = 1;
    double xi = -c;
    double yi = 0;
    double ri = 0.05;
    double Ri = 0.6;
    double xj = c;
    double yj = 0;
    double rj = 0.1;
    double Rj = 0.8;
    double ai = ri + rj + Rj - Ri;

    /* Create parameter t vector */
    size_t Nt = 100;
    std::vector<double> t (Nt, 0);
    double tmax = std::ceil( std::acosh(5/std::abs(ai)) );
    double dt = 2*tmax/(Nt-1);
    for (size_t i=0; i<Nt; i++) {
        t[i] = -tmax + dt*i;
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
    fp = std::fopen("bin/Jn_values.txt", "w");
    if (fp != NULL) {
        for (size_t i=0; i<Nt; i++) {
            std::fprintf(fp, "% lf % lf % lf % lf % lf\n", t[i], Jni_x[i], Jni_y[i], Jnj_x[i], Jnj_y[i]);
        }
    }
    std::fclose(fp);

    return 0;
}
