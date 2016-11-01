/*
	Copyright (C) 2016 Sotiris Papatheodorou

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
            NPFLOAT view_angle;
            NPFLOAT quality;
            np::Circle sensing;
            np::Polygon sensing_poly;
            np::Polygon cell;

            NPFLOAT get_quality() const;
            NPFLOAT get_dquality() const;
            void set_quality();
            void set_sensing();
            void set_sensing_poly();
    };


}


#endif
