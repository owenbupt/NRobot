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

#ifndef __NRobot_hpp
#define __NRobot_hpp

/* Include all NRobot library headers */
#include "NRBase.hpp"
#include "NRPart.hpp"
#if NR_PLOT_AVAILABLE
#include "NRPlot.hpp"
#endif

namespace nr {

/*******************************************************/
/********************** MA class ***********************/
/*******************************************************/
/*!
	Mobile Agent class.
*/
class MA {
	public:
		/****** Data members ******/
		size_t ID;
		Point position;
		Orientation attitude;
		double sensing_radius;
		double uncertainty_radius;
		double communication_radius;
		Polygon sensing;
		Polygon cell;
		Polygon rlimited_cell;
		std::vector<MA> neighbors;

		/****** Constructors ******/
		MA();
		MA(
			Point& pos,
			double sradius = 0,
			double uradius = 0,
			double cradius = 0
		);
		MA(
			Point& pos,
			Orientation& att,
			double sradius = 0,
			double uradius = 0,
			double cradius = 0
		);
};




/*******************************************************/
/********************** MAs class **********************/
/*******************************************************/
/*!
	Mobile Agents class.
*/
class MAs: public std::vector<MA> {
	public:
		/****** Data members ******/

		/****** Constructors ******/
		MAs();
		MAs(
			Points& pos,
			std::vector<double>& sradius,
			std::vector<double>& uradius,
			std::vector<double>& cradius
		);
		MAs(
			Points& pos,
			Orientations& att,
			std::vector<double>& sradius,
			std::vector<double>& uradius,
			std::vector<double>& cradius
		);
};





/**********************************************************/
/********************* Main functions *********************/
/**********************************************************/
void info();

}

#endif
