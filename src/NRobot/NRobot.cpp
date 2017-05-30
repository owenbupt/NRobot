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



/*******************************************************/
/********************** MA class ***********************/
/*******************************************************/
nr::MA::MA() {
	this->sensing_radius = 0;
	this->uncertainty_radius = 0;
	this->communication_radius = 0;
	/* The other data members use their default constructors */
}

nr::MA::MA(
	Point& pos,
	double sradius,
	double uradius,
	double cradius
) {
	this->position = pos;
	this->sensing_radius = sradius;
	this->uncertainty_radius = uradius;
	this->communication_radius = cradius;
	/* The other data members use their default constructors */
}

nr::MA::MA(
	Point& pos,
	Orientation& att,
	double sradius,
	double uradius,
	double cradius
) {
	this->position = pos;
	this->attitude = att;
	this->sensing_radius = sradius;
	this->uncertainty_radius = uradius;
	this->communication_radius = cradius;
	/* The other data members use their default constructors */
}






/**********************************************************/
/********************* Main functions *********************/
/**********************************************************/
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
		std::printf("Using SDL version %d.%d.%d\n", sdl_linked_version.major,
			sdl_linked_version.minor, sdl_linked_version.patch);
	#endif
	#if NR_TIME_EXECUTION
		std::printf("Execution is being timed\n");
	#endif
}
