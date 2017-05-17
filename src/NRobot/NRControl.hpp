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

#ifndef __NRControl_h
#define __NRControl_h

#include <vector>
#include <NPart/NPBase.hpp>
#include "NRBase.hpp"

namespace nr {

    np::Point YS_uniform_quality_control(const np::Polygon& region, const std::vector<MAA>& robots, const size_t i, const bool *neighbors);
    /* Control law used for planar coverage with downwards facing cameras on flying agents */

}

#endif
