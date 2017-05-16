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

#include <cmath>
#include <cfloat>
#include <cstdio>
#include <iostream>
#include <vector>
#include <fstream>

#include "NBase.hpp"


/*******************************************************/
/********************* Point class *********************/
/*******************************************************/
n::Point::Point( double x, double y, double z ) {
	this->x = x;
	this->y = y;
	this->z = z;
}




/*********************************************************/
/********************* Polygon class *********************/
/*********************************************************/
n::Polygon::Polygon() {
	this->contour.resize(0);
	this->is_hole.resize(0);
	this->is_open.resize(0);
}

n::Polygon::Polygon( const n::Point& P ) {
	this->contour.resize(1);
	this->is_hole.resize(1);
	this->is_open.resize(1);
	this->contour[0].push_back(P);
	this->is_hole[0] = false;
	this->is_open[0] = true;
}

n::Polygon::Polygon( const n::Contour& C ) {
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

n::Polygon::Polygon( const n::Circle& C, size_t points_per_circle ) {
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
}




/**********************************************************/
/********************* Polygons class *********************/
/**********************************************************/
n::Polygons::Polygons() {
	this->resize(0);
}

n::Polygons::Polygons( const n::Circles& C, size_t points_per_circle ) {
	this->resize(C.size());
	for (size_t i=0; i<C.size(); i++) {
		this->at(i) = n::Polygon( C[i], points_per_circle );
	}
}




/********************************************************/
/********************* Circle class *********************/
/********************************************************/
n::Circle::Circle() {
	this->center = n::Point();
	this->radius = 0;
}

n::Circle::Circle( const n::Point& P, double r ) {
	this->center.x = P.x;
	this->center.y = P.y;
	this->radius = r;
}




/*********************************************************/
/********************* Non Members **********************/
/*********************************************************/

/****** Operator overloads ******/

bool n::operator == ( const n::Point& A, const n::Point& B ) {
	if ( (std::abs(A.x - B.x) <= N_CMP_ERR) &&
		(std::abs(A.y - B.y) <= N_CMP_ERR) &&
		(std::abs(A.z - B.z) <= N_CMP_ERR) ) {
		return true;
	} else {
		return false;
	}
}

bool n::operator != ( const n::Point& A, const n::Point& B ) {
	if ( (std::abs(A.x - B.x) <= N_CMP_ERR) &&
		(std::abs(A.y - B.y) <= N_CMP_ERR) &&
		(std::abs(A.z - B.z) <= N_CMP_ERR) ) {
		return false;
	} else {
		return true;
	}
}

n::Point n::operator + ( const n::Point& A, const n::Point& B ) {
	return n::Point( A.x+B.x, A.y+B.y, A.z+B.z );
}

n::Point n::operator - ( const n::Point& A, const n::Point& B ) {
	return n::Point( A.x-B.x, A.y-B.y, A.z-B.z );
}

n::Point n::operator - ( const n::Point& A ) {
	return n::Point( -A.x, -A.y, -A.z );
}

void n::operator += ( n::Point& A, const n::Point& B ) {
	A.x += B.x;
	A.y += B.y;
	A.z += B.z;
}

void n::operator -= ( n::Point& A, const n::Point& B ) {
	A.x -= B.x;
	A.y -= B.y;
	A.z -= B.z;
}

void n::operator + ( n::Polygon& P, const n::Point& A ) {
	n::translate( &P, A );
}

void n::operator + ( const n::Point& A, n::Polygon& P ) {
	n::translate( &P, A );
}

void n::operator - ( n::Polygon& P, const n::Point& A ) {
	n::translate( &P, -A );
}

std::ostream& n::operator << ( std::ostream& output, const n::Point& P ) {
	output << P.x << " " << P.y << " " << P.z;
	return output;
}

std::ostream& n::operator << ( std::ostream& output, const n::Contour& C ) {
	for (size_t i=0; i<C.size(); i++) {
		output << C.at(i) << "\n";
	}
	return output;
}

/****** Point ******/
double n::norm( const n::Point& A ) {
	return sqrt( pow(A.x, 2) + pow(A.y, 2) + pow(A.z, 2) );
}

double n::dist( const n::Point& A, const n::Point& B ) {
	return sqrt( pow(A.x-B.x, 2) + pow(A.y-B.y, 2) + pow(A.z-B.z, 2) );
}

double n::dist( const n::Point& A, const n::Contour& C ) {
	return 0;
}

double n::dist( const n::Point& A, const n::Polygon& P ) {
	return 0;
}

double n::dot( const n::Point& A, const n::Point& B ) {
	return A.x*B.x + A.y*B.y + A.z*B.z;
}

n::Point n::rotate( const n::Point& A, double theta ) {
	Point P;
	P.x = std::cos(theta)*A.x - std::sin(theta)*A.y;
	P.y = std::sin(theta)*A.x + std::cos(theta)*A.y;

	return P;
}

n::Point n::midpoint( const n::Point& A, const n::Point& B ) {
	return n::Point( (A.x+B.x)/2, (A.y+B.y)/2, (A.z+B.z)/2 );
}

/****** Contour ******/
int n::read( n::Contour* C, const char* fname ) {
	size_t  num_vertices;
	double x, y;

	FILE* fp = std::fopen(fname, "r");
	if (fp != NULL) {

		/* Read vertex number */
		std::fscanf(fp, "%lu", &num_vertices);
		/* Resize vectors */
		C->resize(num_vertices);

		/* Loop over each vertex */
		for (size_t i=0; i<num_vertices; i++) {

			/* Read each vertex */
			std::fscanf(fp, "%lf %lf", &x, &y);
			C->at(i).x = x;
			C->at(i).y = y;
		}

		std::fclose(fp);
		return 0;
	} else {
		std::printf("Contour read error: file %s could not be opened\n", fname);
		C->resize(0);
		return 1;
	}
}

int n::write( const n::Contour& C, const char* fname, const char* mode ) {
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

void n::print( const n::Contour& C ) {

	std::printf("Vertices %lu\n", C.size());
	for (size_t i=0; i<C.size(); i++) {
		std::printf("% .5lf % .5lf\n", (double) C.at(i).x, (double) C.at(i).y);
	}
}

double n::area( const n::Contour& C, bool signed_area ) {
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

n::Point n::centroid( const n::Contour& C ) {
	n::Point centr;
	size_t Nv = C.size();

	for (size_t i=0; i<Nv; i++) {
		size_t ii = (i+1) % Nv;
		centr.x += (C.at(i).x + C.at(ii).x) *
			(C.at(i).x*C.at(ii).y - C.at(ii).x*C.at(i).y);
		centr.y += (C.at(i).y + C.at(ii).y) *
			(C.at(i).x*C.at(ii).y - C.at(ii).x*C.at(i).y);
	}

	/* Get signed area */
	double A = n::area( C, true );
	return 1/(6*A) * centr;
}

bool n::is_CW( const n::Contour& C ) {
	/* Depends on signed area */
	if (n::area( C, true ) < 0) {
		return true;
	} else {
		return false;
	}
}

void n::reverse_order( n::Contour* C ) {
	n::Point tmp;
	size_t Nv = C->size();

	/* Loop over the first half vertices and swap them with the second half */
	for (size_t i=0; i<std::floor( Nv/2 ); i++) {
		tmp = C->at(i);
		C->at(i) = C->at(Nv-1-i);
		C->at(Nv-1-i) = tmp;
	}
}

void n::make_CW( n::Contour* C ) {
	if ( !n::is_CW( *C ) ) {
		n::reverse_order( C );
	}
}

void n::make_CCW( n::Contour* C ) {
	if ( n::is_CW( *C ) ) {
		n::reverse_order( C );
	}
}

/****** Polygon ******/
int n::read( n::Polygon* P, const char* fname, bool read_hole, bool read_open ) {
	size_t num_contours, num_vertices;
	int is_hole, is_open;
	double x, y;

	FILE* fp = fopen(fname, "r");
	if (fp != NULL) {

		/* Read contour number */
		std::fscanf(fp, "%lu", &num_contours);
		/* Resize vectors */
		P->contour.resize(num_contours);
		P->is_hole.resize(num_contours);
		P->is_open.resize(num_contours);

		/* Loop over each contour */
		for (size_t i=0; i<num_contours; i++) {

			/* Read vertex number */
			std::fscanf(fp, "%lu", &num_vertices);
			P->contour[i].resize(num_vertices);

			/* Read hole flag */
			if (read_hole) {
				std::fscanf(fp, "%d", &is_hole);
				P->is_hole[i] = is_hole;
			} else {
				P->is_hole[i] = false;
			}

			/* Read open flag */
			if (read_open) {
				std::fscanf(fp, "%d", &is_open);
				P->is_open[i] = is_open;
			} else {
				P->is_open[i] = false;
			}

			/* Loop over each vertex */
			for (size_t j=0; j<num_vertices; j++) {

				/* Read each vertex */
				std::fscanf(fp, "%lf %lf", &x, &y);
				P->contour[i][j].x = x;
				P->contour[i][j].y = y;
			}

			/* Set contour orientation */
			if (P->is_hole[i]) {
				n::make_CCW( &(P->contour[i]) );
			} else {
				n::make_CW( &(P->contour[i]) );
			}
		}

		std::fclose(fp);
		return 0;
	} else {
		std::printf("Polygon read error: file %s could not be opened\n", fname);
		P->contour.resize(0);
		P->is_hole.resize(0);
		P->is_open.resize(0);
		return 1;
	}
}

int n::write( const n::Polygon& P, const char* fname, bool write_hole, bool write_open, const char* mode ) {
	FILE* fp = std::fopen(fname, mode);
	if (fp != NULL) {

		/* Write contour number */
		std::fprintf(fp, "%lu\n", P.contour.size());

		/* Loop over each contour */
		for (size_t i=0; i<P.contour.size(); i++) {

			/* Write vertex number */
			std::fprintf(fp, "%lu\n", P.contour[i].size());

			/* Write hole flag */
			if (write_hole) {
				std::fprintf(fp, "%d\n", (int) P.is_hole[i]);
			}

			/* Write open flag */
			if (write_open) {
				std::fprintf(fp, "%d\n", (int) P.is_open[i]);
			}

			/* Loop over each vertex */
			for (size_t j=0; j<P.contour[i].size(); j++) {

				/* Write each vertex */
				std::fprintf(fp, "% lf % lf\n",
				(double) P.contour[i][j].x,	(double) P.contour[i][j].y);
			}
		}

		std::fclose(fp);
		return 0;
	} else {
		std::printf("Polygon write error: file %s could not be opened\n", fname);
		return 1;
	}
}

void n::print( const n::Polygon& P ) {
	std::printf("Contours: %lu\n", P.contour.size());
	for (size_t i=0; i<P.contour.size(); i++) {
		std::printf("Contour %lu: Hole %d, Open %d, Vertices %lu\n",
			i, (int) P.is_hole[i], (int) P.is_open[i], P.contour[i].size());

		for (size_t j=0; j<P.contour[i].size(); j++) {
			std::printf("% .20lf % .20lf\n",
				(double) P.contour[i][j].x, (double) P.contour[i][j].y);
		}
	}
}

double n::diameter( const n::Polygon& P ) {
	/* Put all polygon vertices in a single contour */
	n::Contour verts;
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
				if (dist(verts.at(i), verts.at(j)) > d ) {
					d = dist(verts.at(i), verts.at(j));
				}
			}
		}
	}

	return d;
}

double n::area( const n::Polygon& P ) {
	double A = 0;

	/* Loop over all contours */
	for (size_t i=0; i<P.contour.size(); i++) {
		/* If the contour is a hole, subtract its area */
		if (P.is_hole[i] && !P.is_open[i]) {
			A -= n::area(P.contour[i]);
		/* If the contour is not a hole, add its area */
		} else if (!P.is_hole[i] && !P.is_open[i]) {
			A += n::area(P.contour[i]);
		}
	}

	return A;
}

n::Point n::centroid( const n::Polygon& P ) {
	/* https://math.stackexchange.com/questions/623841/finding-centroid-of-polygon-with-holes-polygons */

	n::Point C;

	/* Loop over all contours */
	for (size_t i=0; i<P.contour.size(); i++) {
		/* If the contour is a hole, subtract its weighted centroid */
		if (P.is_hole[i] && !P.is_open[i]) {
			C -= n::area(P.contour[i]) * n::centroid(P.contour[i]);
		/* If the contour is not a hole, add its weighted centroid */
		} else if (!P.is_hole[i] && !P.is_open[i]) {
			C += n::area(P.contour[i]) * n::centroid(P.contour[i]);
		}
	}

	return C / n::area(P);
}

n::Point n::normal( const n::Polygon& P, size_t contour, size_t edge ) {
	/* Get the two edge vertices */
	n::Point v1, v2, n;
	v1 = P.contour[contour][edge];
	v2 = P.contour[contour][(edge+1) % P.contour[contour].size()];
	n = v2 - v1;

	/* Assume external contours are CW and internal ones CCW */
	/* Check the orientation of the contour and rotate accordingly */
	if ( n::is_CW(P.contour[contour]) ) {
		/* External contour, rotate 90 degrees */
		n = n::rotate( n, std::asin(1) );
	} else {
		/* Internal contour, rotate -90 degrees */
		n = n::rotate( n, -std::asin(1) );
	}

	/* Make into a unit vector */
	n = n / n::norm(n);

	return n;
}

bool n::is_orientation_correct( const n::Polygon& P ) {
	for (size_t i=0; i<P.contour.size(); i++) {
		if (P.is_hole[i]) {
			if ( n::is_CW(P.contour[i]) ) {
				/* If an internal contour is CW */
				return false;
			}
		} else {
			if ( !n::is_CW(P.contour[i]) ) {
				/* If an external contour is CCW */
				return false;
			}
		}
	}
	return true;
}

bool n::is_point( const n::Polygon& P ) {
	if ( (P.contour.size() == 1) && (P.contour[0].size() == 1) ) {
		return true;
	} else {
		return false;
	}
}

bool n::is_empty( const n::Polygon& P ) {
	if (P.contour.size() == 0) {
		return true;
	} else {
		return false;
	}
}

void n::make_empty( n::Polygon* P ) {
	P->contour.resize(0);
	P->is_hole.resize(0);
	P->is_open.resize(0);
}

void n::fix_orientation( n::Polygon* P, bool follow_hole_flags ) {
	if (follow_hole_flags) {
		/* Set orientation according to hole flags */
		for (size_t i=0; i<P->contour.size(); i++) {
			if (P->is_hole[i]) {
				if ( n::is_CW(P->contour[i]) ) {
					/* If an internal contour is CW */
					n::reverse_order( &(P->contour[i]) );
				}
			} else {
				if ( !n::is_CW(P->contour[i]) ) {
					/* If an external contour is CCW */
					n::reverse_order( &(P->contour[i]) );
				}
			}
		}
	} else {
		/* Set orientation according to contour vertex order */
		for (size_t i=0; i<P->contour.size(); i++) {
			if ( n::is_CW(P->contour[i])) {
				/* Set as external contour */
				P->is_hole[i] = false;
			} else {
				/* Set as interal contour */
				P->is_hole[i] = true;
			}
		}
	}
}

void n::translate( n::Polygon* P, const n::Point& p ) {
	/* Loop over all contours */
	for (size_t i=0; i<P->contour.size(); i++) {
		/* Loop over all vertices */
		for (size_t j=0; j<P->contour[i].size(); j++) {
			/* Translate the vertex */
			P->contour[i].at(j) += p;
		}
	}
}

void n::rotate( n::Polygon* P, double theta, bool around_origin ) {
	n::Point C = n::centroid( *P );

	if (!around_origin) {
		/* Translate the polygon so that its centroid is on the origin */
		n::translate( P, -C );
	}

	/* Loop over all contours */
	for (size_t i=0; i<P->contour.size(); i++) {
		/* Loop over all vertices */
		for (size_t j=0; j<P->contour[i].size(); j++) {
			/* Rotate the vertex */
			P->contour[i][j] = n::rotate( P->contour[i][j], theta );
		}
	}

	if (!around_origin) {
		/* Translate the polygon to its original position */
		n::translate( P, C );
	}
}

n::Contour n::convex_hull( const n::Polygon& P ) {
	/* https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain */
	return n::Contour();
}

/****** Polygons ******/
void n::print( const n::Polygons& P ) {

	std::printf("Polygons: %lu\n", P.size());
	for (size_t k=0; k<P.size(); k++) {
		std::printf("Polygon %lu Contours: %lu\n", k, P.at(k).contour.size());
		for (size_t i=0; i<P.at(k).contour.size(); i++) {
			std::printf("Contour %lu: Hole %d, Open %d\n", i, (int) P.at(k).is_hole[i], (int) P.at(k).is_open[i]);

			for (size_t j=0; j<P.at(k).contour[i].size(); j++) {
				std::printf("% .5lf % .5lf\n", (double) P.at(k).contour[i][j].x, (double) P.at(k).contour[i][j].y);
			}
		}
	}
}

/****** Circle ******/
double n::area( const n::Circle& C ) {
	return M_PI * C.radius * C.radius;
}

bool n::is_point( const n::Circle& C ) {
	if (C.radius == 0) {
		return true;
	} else {
		return false;
	}
}
