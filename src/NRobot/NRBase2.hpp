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

#ifndef __NRBase_h
#define __NRBase_h

#include <NPart/NPBase.hpp>
#include "NRConfig.hpp"

namespace nr {

    /*********************************************************/
    /******************** Base robot class *******************/
    /*********************************************************/
    class Robot_Base {
        public:
            np::Point position;
    };


    /*********************************************************/
    /*************** Mobile Aerial Agent class ***************/
    /*********************************************************/
    class MAA: public Robot_Base {
        public:
            NPFLOAT zmin;
            NPFLOAT zmax;
			/* Maximum and minimum altitude - Set by user */

            NPFLOAT view_angle;
			/* Visual sensor view angle - Set by user */

            NPFLOAT quality;
			/* Coverage quality - Set by set_quality() */

            np::Circle sensing;
			/* Sensing disk - Set by create_sensing_disk() */

            np::Polygon sensing_poly;
			/* Sensing disk as a polygon - Set by create_sensing_poly() */

            np::Polygon cell;
			/* Assigned cell - Set by create_cell() */


            NPFLOAT get_quality() const;
            NPFLOAT get_dquality() const;
            void set_quality();
            void create_sensing_disk();
            void create_sensing_poly();
			void create_cell(const np::Polygon& region, const std::vector<MAA>& robots, size_t i);
			/* i is the index of the current node in robots */
			np::Point get_velocity(const np::Polygon& region, const std::vector<MAA>& robots, size_t i);
			/* Get the velocity produced by the control law */
    };


}


#endif
