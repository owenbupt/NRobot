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

#ifndef __NRControl_hpp
#define __NRControl_hpp

#include "NRBase.hpp"
#include "NRobot.hpp"

namespace nr {

/*******************************************************/
/***************** Space partitioning ******************/
/*******************************************************/
int cell_voronoi( MA* agent, const Polygon& region );
int cell_gvoronoi( MA* agent, const Polygon& region );
int cell_awgvoronoi( MA* agent, const Polygon& region );

/*******************************************************/
/******************** Control laws *********************/
/*******************************************************/
void control_centroid( MA* agent );
void control_free_arc( MA* agent );

} /* End of namespace */

#endif
