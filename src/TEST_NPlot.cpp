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

#include "NPlot.hpp"


int main() {

    n::Points P;
	P.push_back( n::Point(-5,5) );
	P.push_back( n::Point(-5,2) );
	P.push_back( n::Point(-3,5) );
	P.push_back( n::Point(-3,-7) );

    n::Circle C;
	double r = 1.0;
    C = n::Circle(P[0], 1.8*r);
    n::Polygon pC;
	pC = n::Polygon( C );

    /* Plot */
	#if NR_USE_SDL
		if (n::init_SDL()) exit(1);
		PLOT_SCALE = 20;
		bool uquit = false;

		while (!uquit) {
			n::clear_render();
			// n::show_axes();

			PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
			n::plot_points( P );
			n::plot_polygon( pC );
			PLOT_FOREGROUND_COLOR = {0x00, 0xAA, 0x00, 0xFF};

			n::plot_render();
			uquit = n::handle_input();
		}
		n::quit_SDL();
    #endif
}
