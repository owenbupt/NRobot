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

#include <cstdio>
#include <cmath>
#include "NRBase.hpp"



/*********************************************************/
/******************** Base robot class *******************/
/*********************************************************/



/*********************************************************/
/*************** Mobile Aerial Agent class ***************/
/*********************************************************/
NPFLOAT nr::MAA::get_quality() const {
    NPFLOAT z = this->position.z;
    NPFLOAT zmin = this->zmin;
    NPFLOAT zmax = this->zmax;

    if (z < zmin) {
        return 1;
    } else if (z > zmax) {
        return 0;
    } else {
        return std::pow( std::pow( z-zmin ,2) - std::pow( zmax-zmin ,2) ,2) / std::pow( zmax-zmin ,4);
    }
}

NPFLOAT nr::MAA::get_dquality() const {
    NPFLOAT z = this->position.z;
    NPFLOAT zmin = this->zmin;
    NPFLOAT zmax = this->zmax;

    if (z < zmin || z > zmax) {
        return 0;
    } else {
        return  4*(z-zmin)*(std::pow( z-zmin ,2) - std::pow( zmax-zmin ,2)) / std::pow( zmax-zmin ,4);
    }
}

void nr::MAA::set_quality() {
    this->quality = this->get_quality();
}

void nr::MAA::set_sensing() {
    NPFLOAT r = this->position.z * std::tan(this->view_angle/2);
    this->sensing = np::Circle(this->position, r);
}

void nr::MAA::set_sensing_poly() {
    NPFLOAT r = this->position.z * std::tan(this->view_angle/2);
    this->sensing_poly = np::Polygon( np::Circle(this->position, r) );
}
