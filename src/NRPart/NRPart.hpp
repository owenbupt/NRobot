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


#ifndef __NRPart_h
#define __NRPart_h

#include <vector>

#include "NRBase.hpp"


namespace nr {

	void info();
	/* Displays libary info ***** MOVE TO NRobot ***** */

	void voronoi( const Polygon& region, const Points& seeds, Polygons* cells);
	/* Voronoi diagram */

	void g_voronoi( const Polygon& region, const Circles& seeds, Polygons* cells, const size_t points_per_branch = 101);
	/* Guaranteed Voronoi diagram */

	void ys_partitioning( const Polygon& region, const Polygons& seeds, Polygons* cells);
	/* Yannis Stergiopoulos partitioning (ICRA 2014) */

	void ysuq_partitioning( const Polygon& region, const Circles& seeds, const std::vector<double>& quality, Polygons* cells, bool **neighbors = NULL);
	/* Yannis Stergiopoulos uniform quality partitioning (RAS 2017) */

}

#endif
