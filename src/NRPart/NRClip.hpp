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


#ifndef __NRClip_hpp
#define __NRClip_hpp

#include "NRBase.hpp"


#define NR_STRICTLY_SIMPLE false
#define NR_EXPONENT_SCALING 0

namespace nr {
	enum Clip_type {
		AND,
		OR,
		XOR,
		DIFF
	};

	bool polygon_clip_fast(
		Clip_type,
		const Polygon& S1,
		const Polygon& S2,
		Polygon* R
	);
	/*
		Execute a polygon clipping operation between S1 and S2 storing the
		result in R. The resulting polygon's contour orientation is not
		consistent. This function is faster but does not distinguish
		between external and internal contours.
	*/

	bool polygon_clip(
		Clip_type,
		const Polygon& S1,
		const Polygon& S2,
		Polygon* R
	);
	/*
		Execute a polygon clipping operation between S1 and S2 storing the
		result in R. This function is slower but the result has correct
		is_hole and is_open flags for each contour.
	*/
}


#endif
