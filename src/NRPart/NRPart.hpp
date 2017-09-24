/*
 *  Copyright (C) 2016-2017 Sotiris Papatheodorou
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


#ifndef __NRPart_h
#define __NRPart_h

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "NRBase.hpp"


namespace nr {

	int voronoi(
		const Polygon& region,
		const Points& seeds,
		Polygons* cells
	);
	/*
		Voronoi diagram of the points in seeds. The Voronoi cells are stored in
		cells. The cells are constrained inside the polygon region.
	*/

	int voronoi_cell(
		const Polygon& region,
		const Points& seeds,
		const size_t subject,
		Polygon* cell
	);
	/*
		Voronoi cell of point subject in seeds with respect to the other seeds.
		Its Voronoi cell is stored in cell. The cell is constrained inside the
		polygon region.
	*/

	int g_voronoi(
		const Polygon& region,
		const Circles& seeds,
		Polygons* cells,
		const size_t points_per_branch = 101
	);
	/*
		Guaranteed Voronoi diagram of the disks in seeds. The GV cells are
		stored in cells. The cells are constrained inside the polygon region.
		By increasing points_per_branch, the hyperbolic branches of GV cells
		are represented with better accuracy.
	*/

	int g_voronoi_cell(
		const Polygon& region,
		const Circles& seeds,
		const size_t subject,
		Polygon* cell,
		const size_t points_per_branch = 101
	);
	/*
		Guaranteed Voronoi cell of disk subject in seeds. The GV cell is
		stored in cell. The cell is constrained inside the polygon region.
		By increasing points_per_branch, the hyperbolic branches of the GV cell
		are represented with better accuracy.
	*/

	int awg_voronoi_cell(
		const Polygon& region,
		const Circles& seeds,
		const std::vector<double>& weights,
		const size_t subject,
		Polygon* cell,
		const size_t points_per_branch = 101
	);
	/*
		Additively Weighted Guaranteed Voronoi cell of disk subject in seeds.
		The AWGV cell is stored in cell. The cell is constrained inside the
		polygon region. By increasing points_per_branch, the hyperbolic
		branches of the GV cell are represented with better accuracy. The
		weights vector contains the weights of all seed disks.
	*/

	int ys_partitioning(
		const Polygon& region,
		const Polygons& seeds,
		Polygons* cells
	);
	/* Yannis Stergiopoulos partitioning (ICRA 2014) */

	int anisotropic_partitioning_cell(
		const Polygon& region,
		const Polygons& sensing,
		const size_t subject,
		Polygon* cell
	);
	/*
		Yannis Stergiopoulos partitioning (ICRA 2014)
	*/

	int ysuq_partitioning(
		const Polygon& region,
		const Circles& seeds,
		const std::vector<double>& quality,
		Polygons* cells,
		bool **neighbors = NULL
	);
	/* Yannis Stergiopoulos uniform quality partitioning (RAS 2017) */

	int au_partitioning_cell(
		const Polygon& region,
		const Polygons& guaranteed_sensing,
		const Polygons& relaxed_sensing,
		const Polygons& total_sensing,
		const double relaxed_sensing_quality,
		const size_t subject,
		Polygon* cell
	);
	/*
		Partitioning when using anisotropic sensors with positioning and
		orientation uncertainty. (ICRA 2018)
	*/
}

#endif
