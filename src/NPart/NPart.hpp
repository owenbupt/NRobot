/*
	Copyright (C) 2016 Sotiris Papatheodorou

	This file is part of NPart.

    NPart is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    NPart is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NPart.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef __NPart_h
#define __NPart_h

#include <vector>
#include "NPBase.hpp"


namespace np {

	void np_info();

	void voronoi( const Polygon& region, const Points& seeds, Polygons& cells);

	void guaranteed_voronoi( const Polygon& region, const Circles& seeds, Polygons& cells, const size_t points_per_branch = 101);

	void YS_partitioning( const Polygon& region, const Polygons& seeds, Polygons& cells);

	void YS_uniform_quality( const Polygon& region, const Circles& seeds, const std::vector<double>& quality, Polygons& cells, bool **neighbors = NULL);

}

#endif
