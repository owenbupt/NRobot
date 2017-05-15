/*
	Copyright (C) 2016-2017 Sotiris Papatheodorou

	This file is part of NBase.

    NBase is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    NBase is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NBase.  If not, see <http://www.gnu.org/licenses/>.
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

bool n::Point::operator == ( const Point& A ) {
	if ( (abs(this->x - A.x) <= NR_CMP_ERR) &&
		(abs(this->y - A.y) <= NR_CMP_ERR) &&
		(abs(this->z - A.z) <= NR_CMP_ERR) ) {
		return true;
	} else {
		return false;
	}
}

bool n::Point::operator != ( const Point& A ) {
	if ( (abs(this->x - A.x) <= NR_CMP_ERR) &&
		(abs(this->y - A.y) <= NR_CMP_ERR) &&
		(abs(this->z - A.z) <= NR_CMP_ERR) ) {
		return false;
	} else {
		return true;
	}
}

n::Point n::Point::operator + ( const Point& A ) {
	return Point( this->x+A.x, this->y+A.y, this->z+A.z );
}

n::Point n::Point::operator - ( const Point& A ) {
	return Point( this->x-A.x, this->y-A.y, this->z-A.z );
}

n::Point n::Point::operator - () const {
	return Point( -x, -y, -z );
}

void n::Point::operator += ( const Point& A ) {
	this->x += A.x;
	this->y += A.y;
	this->z += A.z;
}

void n::Point::operator -= ( const Point& A ) {
	this->x -= A.x;
	this->y -= A.y;
	this->z -= A.z;
}




/********************************************************/
/********************* Circle class *********************/
/********************************************************/
n::Circle::Circle() {
	this->center = n::Point();
	this->radius = 0;
}

n::Circle::Circle( n::Point& P, double r ) {
	this->center.x = P.x;
	this->center.y = P.y;
	this->radius = r;
}




// /*********************************************************/
// /********************* Polygon class *********************/
// /*********************************************************/
// np::Polygon::Polygon( ) {}
//
// np::Polygon::Polygon( const char* filename ) {
// 	this->np::Polygon::read( filename );
// }
//
// np::Polygon::Polygon( const n::Point& P ) {
// 	this->contour.resize(1);
// 	this->is_hole.resize(1);
// 	this->is_open.resize(1);
// 	this->contour[0].push_back(P);
// 	this->is_hole[0] = false;
// 	this->is_open[0] = true;
// }
//
// np::Polygon::Polygon( const n::Circle& C, size_t points_per_circle ) {
// 	this->contour.resize(1);
// 	this->is_hole.resize(1);
// 	this->is_open.resize(1);
// 	this->contour[0].resize(points_per_circle);
// 	this->is_hole[0] = false;
// 	this->is_open[0] = false;
//
// 	double dt = 2*M_PI / (points_per_circle);
//
// 	for (size_t i=0; i<points_per_circle; i++) {
// 		this->contour[0][i].x = C.radius * std::cos( i*dt ) + C.center.x;
// 		this->contour[0][i].y = C.radius * std::sin( i*dt ) + C.center.y;
// 	}
// }
//
//
// void n::Polygon::read( const char* fname, bool read_hole, bool read_open ) {
// 	FILE *fp;
// 	size_t num_contours, num_vertices;
// 	int is_hole, is_open;
// 	double x, y;
//
// 	fp = fopen(fname, "r");
// 	if (fp != NULL) {
//
// 		/* Read contour number */
// 		fscanf(fp, "%lu", &num_contours);
// 		/* Resize vectors */
// 		this->contour.resize(num_contours);
// 		this->is_hole.resize(num_contours);
// 		this->is_open.resize(num_contours);
//
// 		/* Loop over each contour */
// 		for (size_t i=0; i<num_contours; i++) {
//
// 			/* Read vertex number */
// 			fscanf(fp, "%lu", &num_vertices);
// 			this->contour[i].resize(num_vertices);
//
// 			/* Read hole flag */
// 			if (read_hole) {
// 				fscanf(fp, "%d", &is_hole);
// 				this->is_hole[i] = is_hole;
// 			} else {
// 				this->is_hole[i] = false;
// 			}
//
// 			/* Read open flag */
// 			if (read_open) {
// 				fscanf(fp, "%d", &is_open);
// 				this->is_open[i] = is_open;
// 			} else {
// 				this->is_open[i] = false;
// 			}
//
// 			/* Loop over each vertex */
// 			for (size_t j=0; j<num_vertices; j++) {
//
// 				/* Read each vertex */
// 				fscanf(fp, "%lf %lf", &x, &y);
// 				this->contour[i][j].x = x;
// 				this->contour[i][j].y = y;
// 			}
//
// 			/* Set contour orientation */
// 			if (this->is_hole[i]) {
// 				this->contour[i].make_CCW();
// 			} else {
// 				this->contour[i].make_CW();
// 			}
// 		}
//
// 		fclose(fp);
// 	} else {
// 		std::cout << "Polygon read error: file could not be opened" << std::endl;
// 	}
// }
//
// void n::Polygon::write( const char* fname, bool write_hole, bool write_open, const char* mode ) {
// 	FILE *fp;
//
// 	fp = fopen(fname, mode);
// 	if (fp != NULL) {
//
// 		/* Write contour number */
// 		fprintf(fp, "%lu\n", this->contour.size());
//
// 		/* Loop over each contour */
// 		for (size_t i=0; i<this->contour.size(); i++) {
//
// 			/* Write vertex number */
// 			fprintf(fp, "%lu\n", this->contour[i].size());
//
// 			/* Write hole flag */
// 			if (write_hole) {
// 				fprintf(fp, "%d\n", (int) this->is_hole[i]);
// 			}
//
// 			/* Write open flag */
// 			if (write_open) {
// 				fprintf(fp, "%d\n", (int) this->is_open[i]);
// 			}
//
// 			/* Loop over each vertex */
// 			for (size_t j=0; j<this->contour[i].size(); j++) {
//
// 				/* Write each vertex */
// 				fprintf(fp, "% lf % lf\n",
// 				(double) this->contour[i][j].x,	(double) this->contour[i][j].y);
// 			}
// 		}
//
// 		fclose(fp);
// 	} else {
// 		std::cout << "Polygon write error: file could not be opened" << std::endl;
// 	}
// }
//
// void n::Polygon::print() const {
//
// 	std::printf("Contours: %lu\n", this->contour.size());
// 	for (size_t i=0; i<this->contour.size(); i++) {
// 		std::printf("Contour %lu: Hole %d, Open %d, Vertices %lu\n",
// 			i, (int) this->is_hole[i], (int) this->is_open[i], this->contour[i].size());
//
// 		for (size_t j=0; j<this->contour[i].size(); j++) {
// 			std::printf("% .20lf % .20lf\n",
// 				(double) this->contour[i][j].x, (double) this->contour[i][j].y);
// 		}
// 	}
// }
//
//
// double n::Polygon::diameter() const {
// 	/* TO DO: use convex hull to reduce complexity */
// 	double d = 0;
// 	np::Contour verts = this->get_vertices();
//
// 	for (size_t i=0; i<verts.size(); i++) {
// 		for (size_t j=0; j<verts.size(); j++) {
// 			if (i != j) {
// 				if (dist(verts.at(i), verts.at(j)) > d ) {
// 					d = dist(verts.at(i), verts.at(j));
// 				}
// 			}
// 		}
// 	}
//
// 	return d;
// }
//
// double n::Polygon::area() const {
// 	/* TO DO: How does it handle non-simple polygons? */
// 	double A = 0;
//
// 	/* Loop over all contours */
// 	for (size_t i=0; i<this->contour.size(); i++) {
// 		/* If the contour is a hole, subtract its area */
// 		if (this->is_hole[i] && !this->is_open[i]) {
// 			A -= this->contour[i].area();
// 		/* If the contour is not a hole, add its area */
// 		} else if (!this->is_hole[i] && !this->is_open[i]) {
// 			A += this->contour[i].area();
// 		}
// 	}
//
// 	return A;
// }
//
// np::Point n::Polygon::centroid() const {
// 	/* TO DO: How does it handle non-simple polygons? */
// 	/* https://math.stackexchange.com/questions/623841/finding-centroid-of-polygon-with-holes-polygons */
//
// 	np::Point C;
//
// 	/* Loop over all contours */
// 	for (size_t i=0; i<this->contour.size(); i++) {
// 		/* If the contour is a hole, subtract its weighted centroid */
// 		if (this->is_hole[i] && !this->is_open[i]) {
// 			C -= this->contour[i].area() * this->contour[i].centroid();
// 		/* If the contour is not a hole, add its weighted centroid */
// 		} else if (!this->is_hole[i] && !this->is_open[i]) {
// 			C += this->contour[i].area() * this->contour[i].centroid();
// 		}
// 	}
//
// 	return C / this->area();
// }
//
// np::Point n::Polygon::normal(size_t& c, size_t& e) const {
// 	/* Get the two edge vertices */
// 	np::Point v1, v2, n;
// 	v1 = this->contour[c][e];
// 	v2 = this->contour[c][(e+1) % this->contour[c].size()];
// 	n = v2 - v1;
//
// 	/* Assume external contours are CW and internal ones CCW */
// 	/* Check the orientation of the contour and rotate accordingly */
// 	if (this->contour[c].is_CW()) {
// 		/* External contour, rotate 90 degrees */
// 		n.rotate( std::asin(1) );
// 	} else {
// 		/* Internal contour, rotate -90 degrees */
// 		n.rotate( -std::asin(1) );
// 	}
//
// 	/* Make into a unit vector */
// 	n = n / n.norm();
//
// 	return n;
// }
//
//
// bool n::Polygon::correct_orientation() const {
// 	for (size_t i=0; i<this->contour.size(); i++) {
// 		if (this->is_hole[i]) {
// 			if (this->contour[i].is_CW()) {
// 				/* If an internal contour is CW */
// 				return false;
// 			}
// 		} else {
// 			if (!this->contour[i].is_CW()) {
// 				/* If an external contour is CCW */
// 				return false;
// 			}
// 		}
// 	}
// 	return true;
// }
//
// bool n::Polygon::is_point() const {
// 	if ( (this->contour.size() == 1) && (this->contour[0].size() == 1) ) {
// 		return true;
// 	} else {
// 		return false;
// 	}
// }
//
// bool n::Polygon::is_empty() const {
// 	if (this->contour.size() == 0) {
// 		return true;
// 	} else {
// 		return false;
// 	}
// }
//
// void n::Polygon::make_empty() {
// 	this->contour.resize(0);
// 	this->is_hole.resize(0);
// 	this->is_open.resize(0);
// }
//
// void n::Polygon::fix_orientation( bool follow_hole_flags ) {
// 	if (follow_hole_flags) {
// 		/* Set orientation according to hole flags */
// 		for (size_t i=0; i<this->contour.size(); i++) {
// 			if (this->is_hole[i]) {
// 				if (this->contour[i].is_CW()) {
// 					/* If an internal contour is CW */
// 					this->contour[i].reverse_order();
// 				}
// 			} else {
// 				if (!this->contour[i].is_CW()) {
// 					/* If an external contour is CCW */
// 					this->contour[i].reverse_order();
// 				}
// 			}
// 		}
// 	} else {
// 		/* Set orientation according to contour vertex order */
// 		for (size_t i=0; i<this->contour.size(); i++) {
// 			if (this->contour[i].is_CW()) {
// 				/* Set as external contour */
// 				this->is_hole[i] = false;
// 			} else {
// 				/* Set as interal contour */
// 				this->is_hole[i] = true;
// 			}
// 		}
// 	}
// }
//
//
// void n::Polygon::translate( const Point& P ) {
// 	/* Loop over all contours */
// 	for (size_t i=0; i<this->contour.size(); i++) {
// 		/* Loop over all vertices */
// 		for (size_t j=0; j<this->contour[i].size(); j++) {
// 			/* Translate the vertex */
// 			this->contour[i].at(j) += P;
// 		}
// 	}
// }
//
// void n::Polygon::rotate( const double& theta, const bool around_origin ) {
// 	np::Point C = this->centroid();
//
// 	if (!around_origin) {
// 		/* Translate the polygon so that its centroid is on the origin */
// 		this->translate(-C);
// 	}
//
// 	/* Loop over all contours */
// 	for (size_t i=0; i<this->contour.size(); i++) {
// 		/* Loop over all vertices */
// 		for (size_t j=0; j<this->contour[i].size(); j++) {
// 			/* Rotate the vertex */
// 			this->contour[i][j].rotate(theta);
// 		}
// 	}
//
// 	if (!around_origin) {
// 		/* Translate the polygon to its original position */
// 		this->translate(C);
// 	}
// }
//
//
// bool n::Polygon::contains( const Point& ) const {
// 	return false;
// }
//
// np::Polygon n::Polygon::convex_hull() const {
// 	/* https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain */
// 	return n::Polygon();
// }
//
//
// /*----- Private Members -----*/
// np::Contour n::Polygon::get_vertices() const {
// 	/* Find the total number of vertices */
// 	size_t Nv = 0;
// 	for (size_t i=0; i<this->contour.size(); i++) {
// 		Nv += this->contour[i].size();
// 	}
//
// 	/* Create destination vector */
// 	np::Contour verts;
// 	verts.resize(Nv);
//
// 	/* Copy vertices to the vector */
// 	size_t k = 0;
// 	for (size_t i=0; i<this->contour.size(); i++) {
// 		for (size_t j=0; j<this->contour[i].size(); j++) {
// 			verts.at(k++) = this->contour[i].at(j);
// 		}
// 	}
//
// 	return verts;
// }
//
//
// /*----- Non Members -----*/
// void n::operator + ( n::Polygon& P, const n::Point& A ) {
// 	P.translate(A);
// }
//
// void n::operator + ( const n::Point& A, n::Polygon& P ) {
// 	P.translate(A);
// }
//
// void n::operator - ( n::Polygon& P, const n::Point& A ) {
// 	P.translate(-A);
// }
//
//
//
//
//
//
//
// /**********************************************************/
// /********************* Polygons class *********************/
// /**********************************************************/
// np::Polygons::Polygons() {}
//
// np::Polygons::Polygons( const n::Circles& C, size_t points_per_circle ) {
// 	this->resize(C.size());
// 	for (size_t i=0; i<C.size(); i++) {
// 		this->at(i) = n::Polygon( C[i], points_per_circle );
// 	}
// }
//
//
// void n::Polygons::print() const {
//
// 	std::printf("Polygons: %lu\n", this->size());
// 	for (size_t k=0; k<this->size(); k++) {
// 		std::printf("Polygon %lu Contours: %lu\n", k, this->at(k).contour.size());
// 		for (size_t i=0; i<this->at(k).contour.size(); i++) {
// 			std::printf("Contour %lu: Hole %d, Open %d\n",
// 				i, (int) this->at(k).is_hole[i], (int) this->at(k).is_open[i]);
//
// 			for (size_t j=0; j<this->at(k).contour[i].size(); j++) {
// 				std::printf("% .5lf % .5lf\n",
// 					(double) this->at(k).contour[i][j].x, (double) this->at(k).contour[i][j].y);
// 			}
// 		}
// 	}
// }




/*********************************************************/
/********************* Non Members **********************/
/*********************************************************/

/****** Point ******/
double n::norm( n::Point& A ) {
	return sqrt( pow(A.x, 2) + pow(A.y, 2) + pow(A.z, 2) );
}

double n::dist( n::Point& A, n::Point& B ) {
	return sqrt( pow(A.x-B.x, 2) + pow(A.y-B.y, 2) + pow(A.z-B.z, 2) );
}

double n::dist( n::Point& A, n::Contour& C ) {
	return 0;
}

double n::dist( n::Point& A, n::Polygon& P ) {
	return 0;
}

double n::dot( n::Point& A, n::Point& B ) {
	return A.x*B.x + A.y*B.y + A.z*B.z;
}

n::Point n::rotate( n::Point& A, double theta ) {
	Point P;
	P.x = std::cos(theta)*A.x - std::sin(theta)*A.y;
	P.y = std::sin(theta)*A.x + std::cos(theta)*A.y;

	return P;
}

n::Point n::midpoint( n::Point& A, n::Point& B ) {
	return n::Point( (A.x+B.x)/2, (A.y+B.y)/2, (A.z+B.z)/2 );
}

/****** Contour ******/
int n::read( n::Contour& C, const char* fname ) {
	FILE *fp;
	size_t  num_vertices;
	double x, y;

	fp = std::fopen(fname, "r");
	if (fp != NULL) {

		/* Read vertex number */
		if ( std::fscanf(fp, "%lu", &num_vertices) ) {
			std::printf("Contour read error: file %s could not be opened\n", fname);
			C.resize(0);
			return 1;
		}
		/* Resize vectors */
		C.resize(num_vertices);

		/* Loop over each vertex */
		for (size_t i=0; i<num_vertices; i++) {

			/* Read each vertex */
			if ( std::fscanf(fp, "%lf %lf", &x, &y) ) {
				std::printf("Contour read error: file %s could not be opened\n", fname);
				C.resize(0);
				return 1;
			}
			C.at(i).x = x;
			C.at(i).y = y;
		}

		std::fclose(fp);
		return 0;
	} else {
		std::printf("Contour read error: file %s could not be opened\n", fname);
		C.resize(0);
		return 1;
	}
}

int n::write( n::Contour& C, const char* fname, const char* mode ) {
	FILE *fp;

	fp = std::fopen(fname, mode);
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
		C.resize(0);
		return 1;
	}
}

void n::print( n::Contour& C ) {

	std::printf("Vertices %lu\n", C.size());
	for (size_t i=0; i<C.size(); i++) {
		std::printf("% .5lf % .5lf\n", (double) C.at(i).x, (double) C.at(i).y);
	}
}

double n::area( n::Contour& C, bool signed_area ) {
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

n::Point n::centroid( n::Contour& C ) {
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

bool n::is_CW( n::Contour& C ) {
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

/****** Circle ******/
double n::area( n::Circle& C ) {
	return M_PI * C.radius * C.radius;
}

bool n::is_point( n::Circle& C ) {
	if (C.radius == 0) {
		return true;
	} else {
		return false;
	}
}
