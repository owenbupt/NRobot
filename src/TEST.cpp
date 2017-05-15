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

#include "NBase.hpp"
#include <cstdio>
#include <iostream>

using namespace std;

int main() {
    cout << "Testing Point class\n";
    n::Point P;
    n::Point Q(1, 2, 3);
    n::Point R;
    cout << "\tConstructor P\t\t" << P << "\n";
    cout << "\tConstructor Q\t\t" << Q << "\n";
    cout << "\tnorm P\t\t\t" << n::norm(P) << "\n";
    cout << "\tnorm Q\t\t\t" << n::norm(Q) << "\n";
    cout << "\tP == Q\t\t\t" << (P==Q)<< "\n";
    cout << "\tP != Q\t\t\t" << (P!=Q) << "\n";
    cout << "\tP + Q\t\t\t" << (P+Q) << "\n";
    cout << "\tP - Q\t\t\t" << (P-Q) << "\n";
    cout << "\t-P\t\t\t" << (-P) << "\n";
    cout << "\t-Q\t\t\t" << (-Q) << "\n";
    R = P; R+=Q;
    cout << "\tP += Q\t\t\t" << R << "\n";
    R = P; R-=Q;
    cout << "\tP -= Q\t\t\t" << R << "\n";
    cout << "\t2*P\t\t\t" << (2*P)<< "\n";
    cout << "\tQ*2\t\t\t" << (Q*2)<< "\n";
    cout << "\t-2*Q\t\t\t" << (-2*Q)<< "\n";
    cout << "\tQ/2\t\t\t" << (Q/2)<< "\n";
    cout << "Testing non-member functions\n";
    cout << "\tdist(P,Q)\t\t" << n::dist(P,Q) << "\n";
    cout << "\tdot(P,Q)\t\t" << n::dot(P,Q) << "\n";
    cout << "\tmidpoint(P,Q)\t\t" << n::midpoint(P,Q) << "\n";
    cout << endl;


    return 0;
}
