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

#include <cstdio>
#include "NRBase.hpp"
#if NR_PLOT_AVAILABLE
#include "NRPlot.hpp"
#endif


int main() {

    nr::Points P;
	P.push_back( nr::Point(-5,5) );
	P.push_back( nr::Point(-5,2) );
	P.push_back( nr::Point(-3,5) );
	P.push_back( nr::Point(-3,-7) );

    nr::Circle C;
	double r = 1.0;
    C = nr::Circle(P[0], 1.8*r);
    nr::Polygon pC;
	pC = nr::Polygon( C );

    /* Plot */
	#if NR_PLOT_AVAILABLE
		if (nr::plot_init()) exit(1);
		PLOT_SCALE = 20;
		bool uquit = false;

		while (!uquit) {
			nr::plot_clear_render();
			nr::plot_show_axes();

			PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
			nr::plot_points( P );
            PLOT_FOREGROUND_COLOR = {0x00, 0xAA, 0x00, 0xFF};
			nr::plot_polygon( pC );
            PLOT_FOREGROUND_COLOR = {0xAA, 0x00, 0x00, 0xFF};
            nr::plot_polygon_vertices( pC );

			nr::plot_render();
			uquit = nr::plot_handle_input();
		}
		nr::plot_quit();
    #else
        std::printf("SDL 2 is required for NRPlot\n");
    #endif
}
