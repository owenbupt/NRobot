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


#ifndef __NPClip_h
#define __NPClip_h

#include "NPBase.hpp"


#define STRICTLY_SIMPLE false

namespace npclip {
	enum Clip_t{
		AND,
		OR,
		XOR,
		DIFF
	};

	/* Execute a polygon clipping operation between S1 and S2 storing the
		result in R. The resulting polygon's contour orientation is not
		consistent. This function is faster but does not distinguish
		between external and internal contours. */
	bool polygon_clip_fast(
		Clip_t,											/* Operation type */
		const np::Polygon& S1, const np::Polygon& S2,	/* Subject polygons */
		np::Polygon& R									/* Result polygon */
		);

	/* Execute a polygon clipping operation between S1 and S2 storing the
		result in R. This function is slower but the result has correct
		is_hole and is_open flags for each contour. */
	bool polygon_clip(
		Clip_t,											/* Operation type */
		const np::Polygon& S1, const np::Polygon& S2,	/* Subject polygons */
		np::Polygon& R									/* Result polygon */
		);
}


#endif
