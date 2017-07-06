/* This file was created by SymPy 1.0 on 2017-07-06 21:38:40.869727 */

#include <cmath>

#include "J.hpp"

using namespace std;

double nr_FJi_x_xi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {
	return (-xi + xj)*(xi - xj)*(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj)*cosh(t)/pow(pow(xi - xj, 2) + pow(yi - yj, 2), 3.0L/2.0L) - (-xi + xj)*(yi - yj)*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))*sinh(t)/pow(pow(xi - xj, 2) + pow(yi - yj, 2), 3.0L/2.0L) - ((1.0L/4.0L)*xi - 1.0L/4.0L*xj)*(yi - yj)*sinh(t)/(sqrt(pow(xi - xj, 2) + pow(yi - yj, 2))*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))) + 1.0L/2.0L + (-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj)*cosh(t)/sqrt(pow(xi - xj, 2) + pow(yi - yj, 2));
}

double nr_FJi_x_yi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {
	return (xi - xj)*(-yi + yj)*(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj)*cosh(t)/pow(pow(xi - xj, 2) + pow(yi - yj, 2), 3.0L/2.0L) - (-yi + yj)*(yi - yj)*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))*sinh(t)/pow(pow(xi - xj, 2) + pow(yi - yj, 2), 3.0L/2.0L) - ((1.0L/4.0L)*yi - 1.0L/4.0L*yj)*(yi - yj)*sinh(t)/(sqrt(pow(xi - xj, 2) + pow(yi - yj, 2))*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))) - sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))*sinh(t)/sqrt(pow(xi - xj, 2) + pow(yi - yj, 2));
}

double nr_FJi_y_xi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {
	return (-xi + xj)*(xi - xj)*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))*sinh(t)/pow(pow(xi - xj, 2) + pow(yi - yj, 2), 3.0L/2.0L) + (-xi + xj)*(yi - yj)*(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj)*cosh(t)/pow(pow(xi - xj, 2) + pow(yi - yj, 2), 3.0L/2.0L) + ((1.0L/4.0L)*xi - 1.0L/4.0L*xj)*(xi - xj)*sinh(t)/(sqrt(pow(xi - xj, 2) + pow(yi - yj, 2))*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))) + sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))*sinh(t)/sqrt(pow(xi - xj, 2) + pow(yi - yj, 2));
}

double nr_FJi_y_yi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {
	return (xi - xj)*(-yi + yj)*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))*sinh(t)/pow(pow(xi - xj, 2) + pow(yi - yj, 2), 3.0L/2.0L) + (xi - xj)*((1.0L/4.0L)*yi - 1.0L/4.0L*yj)*sinh(t)/(sqrt(pow(xi - xj, 2) + pow(yi - yj, 2))*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))) + (-yi + yj)*(yi - yj)*(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj)*cosh(t)/pow(pow(xi - xj, 2) + pow(yi - yj, 2), 3.0L/2.0L) + 1.0L/2.0L + (-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj)*cosh(t)/sqrt(pow(xi - xj, 2) + pow(yi - yj, 2));
}
double nr_FJj_x_xi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {
	return pow(-xi + xj, 2)*(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj - 1.0L/2.0L*ri - 1.0L/2.0L*rj)*cosh(t)/pow(pow(-xi + xj, 2) + pow(-yi + yj, 2), 3.0L/2.0L) - (-xi + xj)*(-yi + yj)*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow((1.0L/2.0L)*Ri - 1.0L/2.0L*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))*sinh(t)/pow(pow(-xi + xj, 2) + pow(-yi + yj, 2), 3.0L/2.0L) - ((1.0L/4.0L)*xi - 1.0L/4.0L*xj)*(-yi + yj)*sinh(t)/(sqrt(pow(-xi + xj, 2) + pow(-yi + yj, 2))*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow((1.0L/2.0L)*Ri - 1.0L/2.0L*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))) + 1.0L/2.0L - (-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj - 1.0L/2.0L*ri - 1.0L/2.0L*rj)*cosh(t)/sqrt(pow(-xi + xj, 2) + pow(-yi + yj, 2));
}

double nr_FJj_x_yi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {
	return (-xi + xj)*(-yi + yj)*(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj - 1.0L/2.0L*ri - 1.0L/2.0L*rj)*cosh(t)/pow(pow(-xi + xj, 2) + pow(-yi + yj, 2), 3.0L/2.0L) - pow(-yi + yj, 2)*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow((1.0L/2.0L)*Ri - 1.0L/2.0L*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))*sinh(t)/pow(pow(-xi + xj, 2) + pow(-yi + yj, 2), 3.0L/2.0L) - (-yi + yj)*((1.0L/4.0L)*yi - 1.0L/4.0L*yj)*sinh(t)/(sqrt(pow(-xi + xj, 2) + pow(-yi + yj, 2))*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow((1.0L/2.0L)*Ri - 1.0L/2.0L*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))) + sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow((1.0L/2.0L)*Ri - 1.0L/2.0L*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))*sinh(t)/sqrt(pow(-xi + xj, 2) + pow(-yi + yj, 2));
}

double nr_FJj_y_xi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {
	return pow(-xi + xj, 2)*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow((1.0L/2.0L)*Ri - 1.0L/2.0L*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))*sinh(t)/pow(pow(-xi + xj, 2) + pow(-yi + yj, 2), 3.0L/2.0L) + (-xi + xj)*((1.0L/4.0L)*xi - 1.0L/4.0L*xj)*sinh(t)/(sqrt(pow(-xi + xj, 2) + pow(-yi + yj, 2))*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow((1.0L/2.0L)*Ri - 1.0L/2.0L*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))) + (-xi + xj)*(-yi + yj)*(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj - 1.0L/2.0L*ri - 1.0L/2.0L*rj)*cosh(t)/pow(pow(-xi + xj, 2) + pow(-yi + yj, 2), 3.0L/2.0L) - sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow((1.0L/2.0L)*Ri - 1.0L/2.0L*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))*sinh(t)/sqrt(pow(-xi + xj, 2) + pow(-yi + yj, 2));
}

double nr_FJj_y_yi( double t, double xi, double yi, double ri, double Ri, double xj, double yj, double rj, double Rj ) {
	return (-xi + xj)*(-yi + yj)*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow((1.0L/2.0L)*Ri - 1.0L/2.0L*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))*sinh(t)/pow(pow(-xi + xj, 2) + pow(-yi + yj, 2), 3.0L/2.0L) + (-xi + xj)*((1.0L/4.0L)*yi - 1.0L/4.0L*yj)*sinh(t)/(sqrt(pow(-xi + xj, 2) + pow(-yi + yj, 2))*sqrt((1.0L/4.0L)*pow(xi - xj, 2) + (1.0L/4.0L)*pow(yi - yj, 2) - pow((1.0L/2.0L)*Ri - 1.0L/2.0L*Rj + (1.0L/2.0L)*ri + (1.0L/2.0L)*rj, 2))) + pow(-yi + yj, 2)*(-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj - 1.0L/2.0L*ri - 1.0L/2.0L*rj)*cosh(t)/pow(pow(-xi + xj, 2) + pow(-yi + yj, 2), 3.0L/2.0L) + 1.0L/2.0L - (-1.0L/2.0L*Ri + (1.0L/2.0L)*Rj - 1.0L/2.0L*ri - 1.0L/2.0L*rj)*cosh(t)/sqrt(pow(-xi + xj, 2) + pow(-yi + yj, 2));
}
