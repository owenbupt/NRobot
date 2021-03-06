/*
 *  Copyright (C) 2016-2017 Sotiris Papatheodorou
 *
 *  This file is part of NRobot.
 *
 *  NRobot is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  NRobot is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with NRobot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "NRBase.hpp"


#define NR_STRICTLY_SIMPLE false /* Experimental feature of clipper */
#define NR_SCALING_FACTOR 15


/*******************************************************/
/********************* Point class *********************/
/*******************************************************/
nr::Point::Point( const double x, const double y, const double z ) {
	this->x = x;
	this->y = y;
	this->z = z;
}




/*********************************************************/
/********************* Polygon class *********************/
/*********************************************************/
nr::Polygon::Polygon() {
	this->contour.resize(0);
	this->is_hole.resize(0);
	this->is_open.resize(0);
}

nr::Polygon::Polygon( const nr::Point& P ) {
	this->contour.resize(1);
	this->is_hole.resize(1);
	this->is_open.resize(1);
	this->contour[0].push_back(P);
	this->is_hole[0] = false;
	this->is_open[0] = true;
}

nr::Polygon::Polygon( const nr::Contour& C ) {
	this->contour.resize(1);
	this->is_hole.resize(1);
	this->is_open.resize(1);
	this->contour[0].resize( C.size() );
	this->is_hole[0] = false;
	this->is_open[0] = false;

	for (size_t i=0; i<C.size(); i++) {
		this->contour[0][i] = C[i];
	}
}

nr::Polygon::Polygon( const nr::Circle& C, size_t points_per_circle ) {
	this->contour.resize(1);
	this->is_hole.resize(1);
	this->is_open.resize(1);
	this->contour[0].resize(points_per_circle);
	this->is_hole[0] = false;
	this->is_open[0] = false;

	double dt = 2*M_PI / (points_per_circle);

	for (size_t i=0; i<points_per_circle; i++) {
		this->contour[0][i].x = C.radius * std::cos( i*dt ) + C.center.x;
		this->contour[0][i].y = C.radius * std::sin( i*dt ) + C.center.y;
	}

	/* Make CW */
	nr::make_CW( &(this->contour[0]) );
}

nr::Polygon::Polygon( const nr::Ellipse& E, size_t points_per_circle ) {
	this->contour.resize(1);
	this->is_hole.resize(1);
	this->is_open.resize(1);
	this->contour[0].resize(points_per_circle);
	this->is_hole[0] = false;
	this->is_open[0] = false;

	double dt = 2*M_PI / (points_per_circle);

	for (size_t i=0; i<points_per_circle; i++) {
		this->contour[0][i].x = E.a * std::cos( i*dt );
		this->contour[0][i].y = E.b * std::sin( i*dt );

		/* Rotate around the origin */
		this->contour[0][i] = nr::rotate( this->contour[0][i], E.theta );

		/* Translate */
		this->contour[0][i] += E.center;
	}

	/* Make CW */
	nr::make_CW( &(this->contour[0]) );
}




/**********************************************************/
/********************* Polygons class *********************/
/**********************************************************/
nr::Polygons::Polygons() {
	this->resize(0);
}

nr::Polygons::Polygons( const nr::Circles& C, size_t points_per_circle ) {
	this->resize(C.size());
	for (size_t i=0; i<C.size(); i++) {
		this->at(i) = nr::Polygon( C[i], points_per_circle );
	}
}




/********************************************************/
/********************* Circle class *********************/
/********************************************************/
nr::Circle::Circle( const nr::Point& center, double r ) {
	this->center = center;
	this->radius = r;
}




/*********************************************************/
/********************* Circles class *********************/
/*********************************************************/
nr::Circles::Circles() {
	this->resize(0);
}

nr::Circles::Circles( const nr::Points& centers, const std::vector<double>& radii ) {
	/* Resize the Circles vector */
	this->resize(centers.size());
	/* Set the center and radius of each circle */
	for (size_t i=0; i<centers.size(); i++) {
		this->at(i).center = centers[i];
		this->at(i).radius = radii[i];
	}
}




/*********************************************************/
/********************* Ellipse class *********************/
/*********************************************************/
nr::Ellipse::Ellipse(
	double a,
	double b,
	const Point& center,
	double theta
) {
	this->a = a;
	this->b = b;
	this->c = std::sqrt( this->a*this->a - this->b*this->b );
	this->eccentricity = this->c/this->a;

	this->theta = theta;
	this->center = center;
	/* Initial foci */
	this->focus1 = nr::Point(this->c,0);
	this->focus2 = nr::Point(this->c,0);
	/* Rotate around the origin */
	this->focus1 = nr::rotate( this->focus1, theta );
	this->focus2 = nr::rotate( this->focus2, theta );
	/* Translate */
	this->focus1 += center;
	this->focus2 += center;
}




/*******************************************************/
/******************** Attitude class *******************/
/*******************************************************/
nr::Orientation::Orientation( double r, double p, double y ) {
	this->roll = r;
	this->pitch = p;
	this->yaw = y;
}





/*******************************************************/
/********************* Non Members *********************/
/*******************************************************/

/***************************** Operator overloads *****************************/

bool nr::operator == ( const nr::Point& A, const nr::Point& B ) {
	if ( (std::abs(A.x - B.x) <= NR_CMP_ERR) &&
		(std::abs(A.y - B.y) <= NR_CMP_ERR) &&
		(std::abs(A.z - B.z) <= NR_CMP_ERR) ) {
		return true;
	} else {
		return false;
	}
}

bool nr::operator != ( const nr::Point& A, const nr::Point& B ) {
	if ( (std::abs(A.x - B.x) <= NR_CMP_ERR) &&
		(std::abs(A.y - B.y) <= NR_CMP_ERR) &&
		(std::abs(A.z - B.z) <= NR_CMP_ERR) ) {
		return false;
	} else {
		return true;
	}
}

nr::Point nr::operator + ( const nr::Point& A, const nr::Point& B ) {
	return nr::Point( A.x+B.x, A.y+B.y, A.z+B.z );
}

nr::Point nr::operator - ( const nr::Point& A, const nr::Point& B ) {
	return nr::Point( A.x-B.x, A.y-B.y, A.z-B.z );
}

nr::Point nr::operator - ( const nr::Point& A ) {
	return nr::Point( -A.x, -A.y, -A.z );
}

void nr::operator += ( nr::Point& A, const nr::Point& B ) {
	A.x += B.x;
	A.y += B.y;
	A.z += B.z;
}

void nr::operator -= ( nr::Point& A, const nr::Point& B ) {
	A.x -= B.x;
	A.y -= B.y;
	A.z -= B.z;
}

bool nr::operator == ( const nr::Orientation& A, const nr::Orientation& B ) {
	if ( (std::abs(A.roll - B.roll) <= NR_CMP_ERR) &&
		(std::abs(A.pitch - B.pitch) <= NR_CMP_ERR) &&
		(std::abs(A.yaw - B.yaw) <= NR_CMP_ERR) ) {
		return true;
	} else {
		return false;
	}
}

bool nr::operator != ( const nr::Orientation& A, const nr::Orientation& B ) {
	if ( (std::abs(A.roll - B.roll) <= NR_CMP_ERR) &&
		(std::abs(A.pitch - B.pitch) <= NR_CMP_ERR) &&
		(std::abs(A.yaw - B.yaw) <= NR_CMP_ERR) ) {
		return false;
	} else {
		return true;
	}
}

nr::Orientation nr::operator + ( const nr::Orientation& A, const nr::Orientation& B ) {
	return nr::Orientation( A.roll+B.roll, A.pitch+B.pitch, A.yaw+B.yaw );
}

nr::Orientation nr::operator - ( const nr::Orientation& A, const nr::Orientation& B ) {
	return nr::Orientation( A.roll-B.roll, A.pitch-B.pitch, A.yaw-B.yaw );
}

nr::Orientation nr::operator - ( const nr::Orientation& A ) {
	return nr::Orientation( -A.roll, -A.pitch, -A.yaw );
}

void nr::operator += ( nr::Orientation& A, const nr::Orientation& B ) {
	A.roll += B.roll;
	A.pitch += B.pitch;
	A.yaw += B.yaw;
}

void nr::operator -= ( nr::Orientation& A, const nr::Orientation& B ) {
	A.roll -= B.roll;
	A.pitch -= B.pitch;
	A.yaw -= B.yaw;
}


void nr::operator + ( nr::Polygon& P, const nr::Point& A ) {
	nr::translate( &P, A );
}

void nr::operator + ( const nr::Point& A, nr::Polygon& P ) {
	nr::translate( &P, A );
}

void nr::operator - ( nr::Polygon& P, const nr::Point& A ) {
	nr::translate( &P, -A );
}

/****************************** Point ******************************/
void nr::print( const nr::Point& A ) {
	std::printf("% .5lf % .5lf % .5lf\n", (double) A.x, (double) A.y, (double) A.z);
}

double nr::norm( const nr::Point& A ) {
	return sqrt( pow(A.x, 2) + pow(A.y, 2) + pow(A.z, 2) );
}

double nr::norm( const nr::Orientation& A ) {
	return sqrt( pow(A.roll, 2) + pow(A.pitch, 2) + pow(A.yaw, 2) );
}

double nr::dist( const nr::Point& A, const nr::Point& B ) {
	return sqrt( pow(A.x-B.x, 2) + pow(A.y-B.y, 2) + pow(A.z-B.z, 2) );
}

double nr::dist( const nr::Point& A, const nr::Contour& C ) {
	return 0;
}

double nr::dist( const nr::Point& A, const nr::Polygon& P ) {
	return 0;
}

double nr::dot( const nr::Point& A, const nr::Point& B ) {
	return A.x*B.x + A.y*B.y + A.z*B.z;
}

nr::Point nr::rotate( const nr::Point& A, double theta ) {
	Point P;
	P.x = std::cos(theta)*A.x - std::sin(theta)*A.y;
	P.y = std::sin(theta)*A.x + std::cos(theta)*A.y;

	return P;
}

nr::Point nr::midpoint( const nr::Point& A, const nr::Point& B ) {
	return nr::Point( (A.x+B.x)/2, (A.y+B.y)/2, (A.z+B.z)/2 );
}

bool nr::in( const nr::Point& A, const nr::Contour& C ) {
	/*
	   Jimenez, Juan Jose, Francisco R. Feito, and Rafael Jesus Segura.
	   "Robust and Optimized Algorithms for the Point‐in‐Polygon Inclusion
	   Test without Pre‐processing."
	   Computer Graphics Forum. Vol. 28. No. 8. Blackwell Publishing Ltd, 2009.

	   https://scholar.google.com/citations?view_op=view_citation&hl=en&user=sMwAdhEAAAAJ&cstart=20&sortby=pubdate&citation_for_view=sMwAdhEAAAAJ:ULOm3_A8WrAC
	*/

	int inc = 0;

	/* Number of contour vertices */
	size_t Ne = C.size();
	/* Translate all contour vertices by minus the point */
	nr::Contour tmpC = C;
	for (size_t k=0; k<Ne; k++) {
		tmpC[k] -= A;
	}
	/* Loop over all edges of contour */
	for (size_t k=0; k<Ne; k++) {
		nr::Point Vi = tmpC[k];
		double xi = Vi.x;
		double yi = Vi.y;
		nr::Point Vj = tmpC[(k+1) % Ne];
		double xj = Vj.x;
		double yj = Vj.y;

		if (xi*xj <= 0) {
			/* reject triangles under x-axis */
			if ((yi >= 0) || (yj >= 0)) {
				if (xi > xj) {
					double a = xi*yj;
					double b = xj*yi;
					if (a > b) {
						if ((xi == 0) || (xj == 0)) {
							/* OVi or OVj */
							inc += 1;
						} else {
							/* OViVj */
							inc += 2;
						}
					} else if (a == b) {
						/* ViVj or Vi or Vj */
						return true;
					}
				} else if (xi < xj) {
					double a = xi*yj;
					double b = xj*yi;
					if (a < b) {
						if ((xi == 0) || (xj == 0)) {
							/* OVi or OVj */
							inc -= 1;
						} else {
							/* OViVj */
							inc -= 2;
						}
					} else if (a == b) {
						/* ViVj or Vi or Vj */
						return true;
					}
				} else if ((yi <= 0) || (yj <= 0)) {
					/* ViVj (xi=xj) */
					return true;
				}
			}
		}
	}

	/* Changed the below test from inc == 2 to abs(inc) == 2 */
	if (std::abs(inc) == 2) {
		return true;
	} else {
		return false;
	}
}

bool nr::on( const nr::Point& A, const nr::Contour& C ) {
	/*
	   Jimenez, Juan Jose, Francisco R. Feito, and Rafael Jesus Segura.
	   "Robust and Optimized Algorithms for the Point‐in‐Polygon Inclusion
	   Test without Pre‐processing."
	   Computer Graphics Forum. Vol. 28. No. 8. Blackwell Publishing Ltd, 2009.
	*/

	int inc = 0;

	/* Number of contour vertices */
	size_t Ne = C.size();
	/* Translate all contour vertices by minus the point */
	nr::Contour tmpC = C;
	for (size_t k=0; k<Ne; k++) {
		tmpC[k] -= A;
	}
	/* Loop over all edges of contour */
	for (size_t k=0; k<Ne; k++) {
		nr::Point Vi = tmpC[k];
		double xi = Vi.x;
		double yi = Vi.y;
		nr::Point Vj = tmpC[(k+1) % Ne];
		double xj = Vj.x;
		double yj = Vj.y;

		if (xi*xj <= 0) {
			/* reject triangles under x-axis */
			if ((yi >= 0) || (yj >= 0)) {
				if (xi > xj) {
					double a = xi*yj;
					double b = xj*yi;
					if (a > b) {
						if ((xi == 0) || (xj == 0)) {
							/* OVi or OVj */
							inc += 1;
						} else {
							/* OViVj */
							inc += 2;
						}
					} else if (a == b) {
						/* ViVj or Vi or Vj */
						return true;
					}
				} else if (xi < xj) {
					double a = xi*yj;
					double b = xj*yi;
					if (a < b) {
						if ((xi == 0) || (xj == 0)) {
							/* OVi or OVj */
							inc -= 1;
						} else {
							/* OViVj */
							inc -= 2;
						}
					} else if (a == b) {
						/* ViVj or Vi or Vj */
						return true;
					}
				} else if ((yi <= 0) || (yj <= 0)) {
					/* ViVj (xi=xj) */
					return true;
				}
			}
		}
	}

	// if (inc == 2) {
	// 	return true;
	// } else {
	// 	return false;
	// }
	return false;
}

bool nr::in( const nr::Point& A, const nr::Polygon& P ) {
	if ( !nr::is_empty(P) ) {
		return nr::in( A, P.contour[0] );
	} else {
		return false;
	}
}

bool nr::on( const nr::Point& A, const nr::Polygon& P ) {
	if ( !nr::is_empty(P) ) {
		return nr::on( A, P.contour[0] );
	} else {
		return false;
	}
}

bool nr::is_vertex_of( const nr::Point& A, const nr::Polygon& P ) {
	/* Loop over all contours of P. */
	size_t Nc = P.contour.size();
	for (size_t c=0; c<Nc; c++) {
		/* Loop over all vertices of the contour. */
		size_t Nv = P.contour[c].size();
		for (size_t v=0; v<Nv; v++) {
			if (A == P.contour[c][v]) {
				return true;
			}
		}
	}

	return false;
}

/****************************** Contour ******************************/
int nr::read( nr::Contour* C, const char* fname ) {
	size_t  num_vertices;
	double x, y;

	FILE* fp = std::fopen(fname, "r");
	if (fp == NULL) {
		std::printf("Contour read error: file %s could not be opened\n", fname);
		C->resize(0);
		return 1;
	}

	/* Read vertex number */
	if ( std::fscanf(fp, "%lu", &num_vertices) != 1 ) {
		std::printf("Contour read error: file %s could not be read\n", fname);
		std::fclose(fp);
		return 1;
	}
	/* Resize vectors */
	C->resize(num_vertices);

	/* Loop over each vertex */
	for (size_t i=0; i<num_vertices; i++) {

		/* Read each vertex */
		if ( std::fscanf(fp, "%lf %lf", &x, &y) != 2 ) {
			std::printf("Contour read error: file %s could not be read\n", fname);
			std::fclose(fp);
			return 1;
		}
		C->at(i).x = x;
		C->at(i).y = y;
	}

	std::fclose(fp);
	return 0;
}

int nr::write( const nr::Contour& C, const char* fname, const char* mode ) {
	FILE* fp = std::fopen(fname, mode);
	if (fp != NULL) {

		/* Write vertex number */
		std::fprintf(fp, "%lu\n", C.size());

		/* Loop over each vertex */
		for (size_t i=0; i<C.size(); i++) {

			/* Write each vertex */
			std::fprintf(fp, "% lf % lf\n", (double) C.at(i).x,	(double) C.at(i).y);
		}

		std::fclose(fp);
		return 0;
	} else {
		std::printf("Contour write error: file %s could not be opened\n", fname);
		return 1;
	}
}

void nr::print( const nr::Contour& C ) {

	std::printf("Vertices %lu\n", C.size());
	for (size_t i=0; i<C.size(); i++) {
		std::printf("% .5lf % .5lf\n", (double) C.at(i).x, (double) C.at(i).y);
	}
}

double nr::area( const nr::Contour& C, bool signed_area ) {
	double A = 0;
	size_t Nv = C.size();

	for (size_t i=0; i<Nv; i++) {
		size_t ii = (i+1) % Nv;
		A += C.at(i).x * C.at(ii).y - C.at(ii).x * C.at(i).y;
	}

	if (signed_area) {
		return 0.5 * A;
	} else {
		return std::abs(0.5 * A);
	}
}

nr::Point nr::centroid( const nr::Contour& C ) {
	nr::Point centr;
	size_t Nv = C.size();

	for (size_t i=0; i<Nv; i++) {
		size_t ii = (i+1) % Nv;
		centr.x += (C.at(i).x + C.at(ii).x) *
			(C.at(i).x*C.at(ii).y - C.at(ii).x*C.at(i).y);
		centr.y += (C.at(i).y + C.at(ii).y) *
			(C.at(i).x*C.at(ii).y - C.at(ii).x*C.at(i).y);
	}

	/* Get signed area */
	double A = nr::area( C, true );
	return 1/(6*A) * centr;
}

bool nr::is_CW( const nr::Contour& C ) {
	/* Depends on signed area */
	if (nr::area( C, true ) < 0) {
		return true;
	} else {
		return false;
	}
}

void nr::reverse_order( nr::Contour* C ) {
	nr::Point tmp;
	size_t Nv = C->size();

	/* Loop over the first half vertices and swap them with the second half */
	for (size_t i=0; i<std::floor( Nv/2 ); i++) {
		tmp = C->at(i);
		C->at(i) = C->at(Nv-1-i);
		C->at(Nv-1-i) = tmp;
	}
}

void nr::make_CW( nr::Contour* C ) {
	if ( !nr::is_CW( *C ) ) {
		nr::reverse_order( C );
	}
}

void nr::make_CCW( nr::Contour* C ) {
	if ( nr::is_CW( *C ) ) {
		nr::reverse_order( C );
	}
}

/****************************** Polygon ******************************/
int nr::read( nr::Polygon* P, const char* fname, bool read_hole, bool read_open ) {
	size_t num_contours, num_vertices;
	int is_hole, is_open;
	double x, y;

	FILE* fp = fopen(fname, "r");
	if (fp == NULL) {
		std::printf("Polygon read error: file %s could not be opened\n", fname);
		P->contour.resize(0);
		P->is_hole.resize(0);
		P->is_open.resize(0);
		return 1;
	}

	/* Read contour number */
	if ( std::fscanf(fp, "%lu", &num_contours) != 1 ) {
		std::printf("Polygon read error: file %s could not be read\n", fname);
		std::fclose(fp);
		return 1;
	}
	/* Resize vectors */
	P->contour.resize(num_contours);
	P->is_hole.resize(num_contours);
	P->is_open.resize(num_contours);

	/* Loop over each contour */
	for (size_t i=0; i<num_contours; i++) {

		/* Read vertex number */
		if ( std::fscanf(fp, "%lu", &num_vertices) != 1 ) {
			std::printf("Polygon read error: file %s could not be read\n", fname);
			std::fclose(fp);
			return 1;
		}
		P->contour[i].resize(num_vertices);

		/* Read hole flag */
		if (read_hole) {
			if ( std::fscanf(fp, "%d", &is_hole) != 1 ) {
				std::printf("Polygon read error: file %s could not be read\n", fname);
				std::fclose(fp);
				return 1;
			}
			P->is_hole[i] = is_hole;
		} else {
			P->is_hole[i] = false;
		}

		/* Read open flag */
		if (read_open) {
			if ( std::fscanf(fp, "%d", &is_open) != 1 ) {
				std::printf("Polygon read error: file %s could not be read\n", fname);
				std::fclose(fp);
				return 1;
			}
			P->is_open[i] = is_open;
		} else {
			P->is_open[i] = false;
		}

		/* Loop over each vertex */
		for (size_t j=0; j<num_vertices; j++) {

			/* Read each vertex */
			if ( std::fscanf(fp, "%lf %lf", &x, &y) != 2 ) {
				std::printf("Polygon read error: file %s could not be read\n", fname);
				std::fclose(fp);
				return 1;
			}
			P->contour[i][j].x = x;
			P->contour[i][j].y = y;
		}

		/* Set contour orientation */
		if (P->is_hole[i]) {
			nr::make_CCW( &(P->contour[i]) );
		} else {
			nr::make_CW( &(P->contour[i]) );
		}
	}

	std::fclose(fp);
	return 0;
}

int nr::write( const nr::Polygon& P, const char* fname, bool write_hole, bool write_open, const char* mode ) {
	FILE* fp = std::fopen(fname, mode);
	if (fp != NULL) {

		nr::write( P, fp, write_hole, write_open );

		std::fclose(fp);
		return nr::SUCCESS;
	} else {
		std::printf("Polygon write error: file %s could not be opened\n", fname);
		return nr::ERROR_FILE;
	}
}

int nr::write(
	const nr::Polygon& P,
	FILE* file,
	bool write_hole,
	bool write_open
) {
	if (file == NULL) {
		return nr::ERROR_FILE;
	}
	/* Write contour number */
	std::fprintf(file, "%lu\n", P.contour.size());
	/* Loop over each contour */
	for (size_t i=0; i<P.contour.size(); i++) {
		/* Write vertex number */
		std::fprintf(file, "%lu\n", P.contour[i].size());
		/* Write hole flag */
		if (write_hole) {
			std::fprintf(file, "%d\n", (int) P.is_hole[i]);
		}
		/* Write open flag */
		if (write_open) {
			std::fprintf(file, "%d\n", (int) P.is_open[i]);
		}
		/* Loop over each vertex */
		for (size_t j=0; j<P.contour[i].size(); j++) {
			/* Write each vertex */
			std::fprintf(file, "% .*f % .*f\n",
			NR_FLOAT_DIGITS, (double) P.contour[i][j].x,
			NR_FLOAT_DIGITS, (double) P.contour[i][j].y);
		}
	}

	return nr::SUCCESS;
}

void nr::print( const nr::Polygon& P ) {
	std::printf("Contours %lu\n", P.contour.size());
	for (size_t i=0; i<P.contour.size(); i++) {
		std::printf("Contour %lu: Hole %d, Open %d, Vertices %lu\n",
			i, (int) P.is_hole[i], (int) P.is_open[i], P.contour[i].size());

		for (size_t j=0; j<P.contour[i].size(); j++) {
			std::printf("% .20lf % .20lf\n",
				(double) P.contour[i][j].x, (double) P.contour[i][j].y);
		}
	}
}

double nr::diameter( const nr::Polygon& P ) {
	/* Put all polygon vertices in a single contour */
	nr::Contour verts;
	/* Find the total number of vertices */
	size_t Nv = 0;
	for (size_t i=0; i<P.contour.size(); i++) {
		Nv += P.contour[i].size();
	}
	/* Resize destination vector */
	verts.resize(Nv);
	/* Copy vertices to the vector */
	size_t k = 0;
	for (size_t i=0; i<P.contour.size(); i++) {
		for (size_t j=0; j<P.contour[i].size(); j++) {
			verts.at(k++) = P.contour[i].at(j);
		}
	}

	/* Find most distant vertex pair */
	double d = 0;
	for (size_t i=0; i<verts.size(); i++) {
		for (size_t j=0; j<verts.size(); j++) {
			if (i != j) {
				if (nr::dist(verts.at(i), verts.at(j)) > d ) {
					d = nr::dist(verts.at(i), verts.at(j));
				}
			}
		}
	}

	return d;
}

double nr::radius( const nr::Polygon& P, const nr::Point& reference_point ) {
	/* Initialize polygon radius. */
	double r = 0;
	/* Loop over all polygon contours. */
	for (size_t c=0; c<P.contour.size(); c++) {
		/* Loop over all contour vertices. */
		for (size_t v=0; v<P.contour[c].size(); v++) {
			/* Compare the distance of the vertex from the reference point with
			   the current polygon radius. */
			double d = nr::dist( reference_point, P.contour[c][v] );
			if (d > r) {
				r = d;
			}
		}
	}

	return r;
}

double nr::area( const nr::Polygon& P ) {
	double A = 0;

	/* Loop over all contours */
	for (size_t i=0; i<P.contour.size(); i++) {
		/* If the contour is a hole, subtract its area */
		if (P.is_hole[i] && !P.is_open[i]) {
			A -= nr::area(P.contour[i]);
		/* If the contour is not a hole, add its area */
		} else if (!P.is_hole[i] && !P.is_open[i]) {
			A += nr::area(P.contour[i]);
		}
	}

	return A;
}

nr::Point nr::centroid( const nr::Polygon& P ) {
	/* https://math.stackexchange.com/questions/623841/finding-centroid-of-polygon-with-holes-polygons */

	nr::Point C;

	/* Loop over all contours */
	for (size_t i=0; i<P.contour.size(); i++) {
		/* If the contour is a hole, subtract its weighted centroid */
		if (P.is_hole[i] && !P.is_open[i]) {
			C -= nr::area(P.contour[i]) * nr::centroid(P.contour[i]);
		/* If the contour is not a hole, add its weighted centroid */
		} else if (!P.is_hole[i] && !P.is_open[i]) {
			C += nr::area(P.contour[i]) * nr::centroid(P.contour[i]);
		}
	}

	return C / nr::area(P);
}

nr::Point nr::normal( const nr::Polygon& P, size_t contour, size_t edge ) {
	/* Get the two edge vertices */
	nr::Point v1, v2, n;
	v1 = P.contour[contour][edge];
	v2 = P.contour[contour][(edge+1) % P.contour[contour].size()];
	n = v2 - v1;

	/* Assume external contours are CW and internal ones CCW */
	/* Check the orientation of the contour and rotate accordingly */
	if ( nr::is_CW(P.contour[contour]) ) {
		/* External contour, rotate 90 degrees */
		n = nr::rotate( n, std::asin(1) );
	} else {
		/* Internal contour, rotate -90 degrees */
		n = nr::rotate( n, -std::asin(1) );
	}

	/* Make into a unit vector */
	n = n / nr::norm(n);

	return n;
}

bool nr::is_orientation_correct( const nr::Polygon& P ) {
	for (size_t i=0; i<P.contour.size(); i++) {
		if (P.is_hole[i]) {
			if ( nr::is_CW(P.contour[i]) ) {
				/* If an internal contour is CW */
				return false;
			}
		} else {
			if ( !nr::is_CW(P.contour[i]) ) {
				/* If an external contour is CCW */
				return false;
			}
		}
	}
	return true;
}

bool nr::is_point( const nr::Polygon& P ) {
	if ( (P.contour.size() == 1) && (P.contour[0].size() == 1) ) {
		return true;
	} else {
		return false;
	}
}

bool nr::is_empty( const nr::Polygon& P ) {
	if (P.contour.size() == 0) {
		return true;
	} else {
		return false;
	}
}

void nr::make_empty( nr::Polygon* P ) {
	P->contour.resize(0);
	P->is_hole.resize(0);
	P->is_open.resize(0);
}

void nr::fix_orientation( nr::Polygon* P, bool follow_hole_flags ) {
	if (follow_hole_flags) {
		/* Set orientation according to hole flags */
		for (size_t i=0; i<P->contour.size(); i++) {
			if (P->is_hole[i]) {
				if ( nr::is_CW(P->contour[i]) ) {
					/* If an internal contour is CW */
					nr::reverse_order( &(P->contour[i]) );
				}
			} else {
				if ( !nr::is_CW(P->contour[i]) ) {
					/* If an external contour is CCW */
					nr::reverse_order( &(P->contour[i]) );
				}
			}
		}
	} else {
		/* Set orientation according to contour vertex order */
		for (size_t i=0; i<P->contour.size(); i++) {
			if ( nr::is_CW(P->contour[i])) {
				/* Set as external contour */
				P->is_hole[i] = false;
			} else {
				/* Set as interal contour */
				P->is_hole[i] = true;
			}
		}
	}
}

void nr::translate( nr::Polygon* P, const nr::Point& p ) {
	/* Loop over all contours */
	for (size_t i=0; i<P->contour.size(); i++) {
		/* Loop over all vertices */
		for (size_t j=0; j<P->contour[i].size(); j++) {
			/* Translate the vertex */
			P->contour[i].at(j) += p;
		}
	}
}

void nr::rotate( nr::Polygon* P, double theta, bool around_origin ) {
	nr::Point C = nr::centroid( *P );

	if (!around_origin) {
		/* Translate the polygon so that its centroid is on the origin */
		nr::translate( P, -C );
	}

	/* Loop over all contours */
	for (size_t i=0; i<P->contour.size(); i++) {
		/* Loop over all vertices */
		for (size_t j=0; j<P->contour[i].size(); j++) {
			/* Rotate the vertex */
			P->contour[i][j] = nr::rotate( P->contour[i][j], theta );
		}
	}

	if (!around_origin) {
		/* Translate the polygon to its original position */
		nr::translate( P, C );
	}
}

void nr::scale( nr::Polygon* P, double scale_factor ) {
	/* Loop over all contours */
	for (size_t i=0; i<P->contour.size(); i++) {
		/* Loop over all vertices */
		for (size_t j=0; j<P->contour[i].size(); j++) {
			/* Rotate the vertex */
			P->contour[i][j] = scale_factor * P->contour[i][j];
		}
	}
}

int nr::offset_in(
	nr::Polygon* P,
	double offset
) {
	/*
	 *  Compute the polygon diameter in order to create halfplanes of proper
	 *  size.
	 */
	double P_diameter = nr::diameter( *P );
	/* Create a copy of P. */
	nr::Polygon P_copy = *P;
	/* Loop over all polygon contours. */
	for (size_t c=0; c<P->contour.size(); c++) {
		/* Loop over all contour edges. */
		size_t Ne = P->contour[c].size();
		for (size_t e=0; e<Ne; e++) {
			/* Get the edge vertices. */
			nr::Point v1 = P->contour[c][e];
			nr::Point v2 = P->contour[c][(e+1) % Ne];
			/* Find the edge midpoint. */
			nr::Point midpt = nr::midpoint(v1, v2);
			/*
			 *  Find a point inside P whose distance from the edge v1v2 at its
			 *  midpoint is equal to 2*offset.
			 */
			nr::Point p = nr::rotate(v2-v1, -M_PI/2) / nr::dist(v1,v2);
			p = midpt + 2*offset * p;

			/* Create a halfplane to subtract from the polygon. */
			nr::Polygon H = nr::halfplane( midpt, p, P_diameter );

			/* Subtract the halfplane from P. */
			int err = nr::polygon_clip( nr::DIFF, P_copy, H, &P_copy );
			if (err) {
				return nr::ERROR_CLIPPING_FAILED;
			}
		}
	}

	/* Copy the result back to P. */
	*P = P_copy;

	return nr::SUCCESS;
}

nr::Contour nr::convex_hull( const nr::Polygon& P ) {
	/* https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain */
	return nr::Contour();
}

/****************************** Polygons ******************************/
void nr::print( const nr::Polygons& P ) {

	std::printf("Polygons %lu\n", P.size());
	for (size_t k=0; k<P.size(); k++) {
		std::printf("Polygon %lu: Contours %lu\n", k, P.at(k).contour.size());
		for (size_t i=0; i<P.at(k).contour.size(); i++) {
			std::printf("Contour %lu: Hole %d, Open %d, Vertices %lu\n",
				i, (int) P.at(k).is_hole[i], (int) P.at(k).is_open[i], P.at(k).contour[i].size());

			for (size_t j=0; j<P.at(k).contour[i].size(); j++) {
				std::printf("% .5lf % .5lf\n", (double) P.at(k).contour[i][j].x, (double) P.at(k).contour[i][j].y);
			}
		}
	}
}

/****************************** Circle ******************************/
double nr::area( const nr::Circle& C ) {
	return M_PI * C.radius * C.radius;
}

bool nr::is_point( const nr::Circle& C ) {
	if (C.radius == 0) {
		return true;
	} else {
		return false;
	}
}

/****************************** Orientation ******************************/
void nr::print( const nr::Orientation& A ) {
	std::printf("% .5lf % .5lf % .5lf\n", (double) A.roll, (double) A.pitch, (double) A.yaw);
}

/****************************** Polygon clipping ******************************/
int nr::polygon_clip_fast(
	nr::Clip_type clip_type,
	const nr::Polygon& S1,
	const nr::Polygon& S2,
	nr::Polygon* R
) {

	/****** Find the appropriate scaling factor ******/
	double sc = std::pow(10,NR_SCALING_FACTOR);

	/****** Create subject Polygon paths from S1 ******/
	ClipperLib::Paths subj( S1.contour.size() );
	for (size_t i=0; i<S1.contour.size(); i++) {
		subj[i].resize( S1.contour[i].size() );
		for (size_t j=0; j<S1.contour[i].size(); j++) {
			subj[i][j].X = sc * S1.contour[i][j].x;
			subj[i][j].Y = sc * S1.contour[i][j].y;
		}
	}

	/****** Create clip Polygon paths from S2 ******/
	ClipperLib::Paths clip( S2.contour.size() );
	for (size_t i=0; i<S2.contour.size(); i++) {
		clip[i].resize( S2.contour[i].size() );
		for (size_t j=0; j<S2.contour[i].size(); j++) {
			clip[i][j].X = sc * S2.contour[i][j].x;
			clip[i][j].Y = sc * S2.contour[i][j].y;
		}
	}

	/****** Initialize Clipper class ******/
	ClipperLib::Clipper clpr;
	clpr.StrictlySimple(NR_STRICTLY_SIMPLE);

	/****** Add the paths/Polygons to the clipper class ******/
	if ( !clpr.AddPaths(subj, ClipperLib::ptSubject, true) ) {
		std::printf("Clipper error: Invalid subject nr::Polygon %p.\n", (void*) &S1);
		return nr::ERROR_INVALID_SUBJECT;
	}
	if ( !clpr.AddPaths(clip, ClipperLib::ptClip, true) ) {
		std::printf("Clipper error: Invalid clip nr::Polygon %p.\n", (void*) &S2);
		return nr::ERROR_INVALID_CLIP;
	}

	/****** Set clipping operation ******/
	ClipperLib::ClipType clipType;
	switch (clip_type) {
		case AND: clipType = ClipperLib::ctIntersection; break;
		case OR: clipType = ClipperLib::ctUnion; break;
		case XOR: clipType = ClipperLib::ctXor; break;
		case DIFF: clipType = ClipperLib::ctDifference; break;
		default: clipType = ClipperLib::ctIntersection;
	}

	/****** Execute clipping ******/
	ClipperLib::Paths result;
	if ( !clpr.Execute( clipType, result ) ) {
		std::printf("Clipper error: nr::Polygon clipping failed.\n");
		return nr::ERROR_CLIPPING_FAILED;
	}

	/****** Create Polygon from paths ******/
	size_t Nc = result.size();
	R->contour.resize(Nc);
	R->is_hole.resize(Nc);
	R->is_open.resize(Nc);
	for (size_t i=0; i<Nc; i++) {
		R->is_hole[i] = false;
		R->is_open[i] = false;

		size_t Nv = result[i].size();
		R->contour[i].resize(Nv);
		for (size_t j=0; j<Nv; j++) {
			R->contour[i][j].x = result[i][j].X / sc;
			R->contour[i][j].y = result[i][j].Y / sc;
		}
	}

	return nr::SUCCESS;
}


int nr::polygon_clip(
	nr::Clip_type clip_type,
	const nr::Polygon& S1,
	const nr::Polygon& S2,
	nr::Polygon* R
) {

	/****** Check for empty inputs ******/
	if (is_empty(S1) && is_empty(S2)) {
		/* Both polygons are empty */
		make_empty(R);
		return nr::SUCCESS;
	} else if (is_empty(S1)) {
		/* Subject polygon is empty */
		switch (clip_type) {
			case AND:
			case DIFF:
				make_empty(R);
				break;
			case OR:
			case XOR:
				*R = S2;
				break;
		}
		return nr::SUCCESS;
	} else if (is_empty(S2)) {
		/* Clip polygon is empty */
		switch (clip_type) {
			case AND:
				make_empty(R);
				break;
			case OR:
			case XOR:
			case DIFF:
				*R = S1;
				break;
		}
		return nr::SUCCESS;
	}
	/* Else continue normally */

	/****** Find the appropriate scaling factor ******/
	double sc = std::pow(10,NR_SCALING_FACTOR);

	/****** Create subject Polygon paths from S1 ******/
	ClipperLib::Paths subj( S1.contour.size() );
	for (size_t i=0; i<S1.contour.size(); i++) {
		subj[i].resize( S1.contour[i].size() );
		for (size_t j=0; j<S1.contour[i].size(); j++) {
			subj[i][j].Y = sc * S1.contour[i][j].y;
			subj[i][j].X = sc * S1.contour[i][j].x;
		}
	}

	/****** Create clip Polygon paths from S2 ******/
	ClipperLib::Paths clip( S2.contour.size() );
	for (size_t i=0; i<S2.contour.size(); i++) {
		clip[i].resize( S2.contour[i].size() );
		for (size_t j=0; j<S2.contour[i].size(); j++) {
			clip[i][j].Y = sc * S2.contour[i][j].y;
			clip[i][j].X = sc * S2.contour[i][j].x;
		}
	}

	/****** Initialize Clipper class ******/
	ClipperLib::Clipper clpr;
	clpr.StrictlySimple(NR_STRICTLY_SIMPLE);

	/****** Add the paths/Polygons to the clipper class ******/
	if ( !clpr.AddPaths(subj, ClipperLib::ptSubject, true) ) {
		std::printf("Clipper error: Invalid subject polygon %p.\n", (void*) &S1);
		// nr::print(S1);
		return nr::ERROR_INVALID_SUBJECT;
	}
	if ( !clpr.AddPaths(clip, ClipperLib::ptClip, true) ) {
		std::printf("Clipper error: Invalid clip polygon %p.\n", (void*) &S2);
		// nr::print(S2);
		return nr::ERROR_INVALID_CLIP;
	}

	/****** Set clipping operation ******/
	ClipperLib::ClipType clipType;
	switch (clip_type) {
		case AND: clipType = ClipperLib::ctIntersection; break;
		case OR: clipType = ClipperLib::ctUnion; break;
		case XOR: clipType = ClipperLib::ctXor; break;
		case DIFF: clipType = ClipperLib::ctDifference; break;
		default: clipType = ClipperLib::ctIntersection;
	}

	/****** Execute clipping ******/
	ClipperLib::PolyTree result;
	if ( !clpr.Execute( clipType, result ) ) {
		std::printf("Clipper error: nr::Polygon clipping failed.\n");
		return nr::ERROR_CLIPPING_FAILED;
	}

	/****** Create Polygon from PolyTree ******/
	R->contour.resize(result.Total());
	R->is_hole.resize(result.Total());
	R->is_open.resize(result.Total());

	ClipperLib::PolyNode *cnode = result.GetFirst();
	for (int i=0; i<result.Total(); i++) {
		size_t Nv = cnode->Contour.size();
		R->is_hole[i] = cnode->IsHole();
		R->is_open[i] = cnode->IsOpen();
		R->contour[i].resize(Nv);

		for (size_t j=0; j<Nv; j++) {
			R->contour[i][j].x = cnode->Contour[j].X / sc;
			R->contour[i][j].y = cnode->Contour[j].Y / sc;
		}

		cnode = cnode->GetNext();
	}

	return nr::SUCCESS;
}

/****************************** Others ******************************/
nr::Point nr::projection(
	const nr::Point& A,
	const nr::Point& B
) {
	return (nr::dot(A,B) / nr::dot(B,B)) * B;
}

nr::Polygon nr::halfplane(
	const nr::Point& A,
	const nr::Point& B,
	double length
) {
	/* Initialize halfplane */
	nr::Polygon H;
	H.contour.resize(1);
	H.is_hole.resize(1);
	H.is_open.resize(1);
	H.is_hole[0] = false;
	H.is_open[0] = false;
	H.contour[0].resize(4);

	/* Create halfplane assuming both points are on x axis with center on origin */
	H.contour[0][0].x = 0;
	H.contour[0][0].y = -length;
	H.contour[0][1].x = -length;
	H.contour[0][1].y = -length;
	H.contour[0][2].x = -length;
	H.contour[0][2].y = length;
	H.contour[0][3].x = 0;
	H.contour[0][3].y = length;

	/* Rotate halfplane */
	double theta = std::atan2(B.y-A.y, B.x-A.x);
	nr::rotate( &H, theta, true);

	/* Translate halfplane */
	nr::translate( &H, nr::midpoint(A,B) );

	return H;
}

std::vector<double> nr::linspace(
	double start,
	double end,
	size_t num
) {
	/* Initialize vector to zeros */
	std::vector<double> v (num, 0);

	/* Calculate the step size */
	double step = (end-start) / (num-1);

	for (size_t i=0; i<num-1; i++) {
		v[i] = start + i*step;
	}
	/* Ensure the last element has exactly the value of end */
	v[num-1] = end;

	return v;
}

nr::Point nr::cart2pol(
	const Point& P
) {
	double r = std::sqrt( P.x*P.x + P.y*P.y );
	double theta = std::atan2( P.y, P.x );
	return nr::Point( r, theta );
}

nr::Point nr::pol2cart(
	const Point& P
) {
	double x = P.x * std::cos(P.y);
	double y = P.x * std::sin(P.y);
	return nr::Point( x, y );
}
