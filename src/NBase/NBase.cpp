/*
	Copyright (C) 2016 Sotiris Papatheodorou

	This file is part of NPart.

    NPart is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    NPart is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NPart.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <cmath>
#include <cfloat>
#include <cstdio>
#include <iostream>
#include <vector>
#include <fstream>
#include "NPBase.hpp"


/*******************************************************/
/********************* Point class *********************/
/*******************************************************/
np::Point::Point() {
	this->x = 0;
	this->y = 0;
	#if NP_3D_POINTS
		this->z = 0;
	#endif
}

np::Point::Point( NPFLOAT X, NPFLOAT Y ) {
	this->x = X;
	this->y = Y;
}

#if NP_3D_POINTS
	np::Point::Point( NPFLOAT X, NPFLOAT Y, NPFLOAT Z ) {
		this->x = X;
		this->y = Y;
		this->z = Z;
	}
#endif


NPFLOAT np::Point::norm() const {
	#if NP_3D_POINTS
		return sqrt( this->x*this->x + this->y*this->y + this->z*this->z );
	#else
		return sqrt( this->x*this->x + this->y*this->y );
	#endif
}

NPFLOAT np::Point::dist( const Point& A ) const {
	#if NP_3D_POINTS
		return sqrt( pow(this->x-A.x, 2) + pow(this->y-A.y, 2) + pow(this->z-A.z, 2) );
	#else
		return sqrt( pow(this->x-A.x, 2) + pow(this->y-A.y, 2) );
	#endif
}

NPFLOAT np::Point::dist( const Contour& A ) const {
	NPFLOAT mind, d;

	/* Initialize minimum distance */
	mind = np::dist(*this, A[0]);

	/* Loop over all contour vertices */
	for (size_t i=1; i<A.size(); i++) {
		d = np::dist(*this, A[i]);
		if (d < mind) {
			mind = d;
		}
	}

	/* Loop over all contour edges */
	for (size_t i=0; i<A.size(); i++) {
		/* Get the endpoints of the current edge */
		Point v1 = A[i];
		Point v2 = A[(i+1) % A.size()];
		/* Check if the closest point is on the current edge */
		Point P = closest_to_line(v1, v2);

		if ((P.x <= std::max(v1.x, v2.x)) && (P.x >= std::min(v1.x, v2.x)) &&
			(P.y <= std::max(v1.y, v2.y)) && (P.y >= std::min(v1.y, v2.y))) {
			/* Find the distance from the current edge */
			d = this->dist_from_line(v1, v2);
			if (d < mind) {
				mind = d;
			}
		}
	}

	/* Return a positive distance if the point is outside the contour and a negative one if it is inside */
	if (this->in(A)) {
		return -mind;
	} else {
		return mind;
	}
}

NPFLOAT np::Point::dist( const Polygon& A ) const {
	NPFLOAT mind, d;

	/* Initialize minimum distance */
	mind = std::abs(this->dist(A.contour[0]));

	/* Loop over all polygon contours */
	for (size_t c=1; c<A.contour.size(); c++) {
		d = this->dist(A.contour[c]);
		if (d < mind) {
			mind = std::abs(d);
		}
	}

	/* Return a positive distance if the point is outside the polygon and a negative one if it is inside */
	if (this->in(A)) {
		return -mind;
	} else {
		return mind;
	}
}

NPFLOAT np::Point::dist_from_line( const Point& A, const Point& B ) const {
	NPFLOAT a,b,c;
	/* Get line coefficients */
	a = A.y - B.y;
	b = B.x - A.x;
	c = - a*A.x - b*A.y;

	return std::abs(a*this->x + b*this->y + c) / std::sqrt(a*a + b*b);
}

NPFLOAT np::Point::dot( const Point& A ) const {
	#if NP_3D_POINTS
		return this->x*A.x + this->y*A.y + this->z*A.z;
	#else
		return this->x*A.x + this->y*A.y;
	#endif
}

np::Point np::Point::midpoint( const Point& A ) const {
	#if NP_3D_POINTS
		return Point( (this->x+A.x)/2, (this->y+A.y)/2, (this->z+A.z)/2 );
	#else
		return Point( (this->x+A.x)/2, (this->y+A.y)/2 );
	#endif
}

np::Point np::Point::closest_to_line( const Point& A, const Point& B ) const {
	NPFLOAT a,b,c;
	/* Get line coefficients */
	a = A.y - B.y;
	b = B.x - A.x;
	c = - a*A.x - b*A.y;

	np::Point P;
	P.x = (b*(b*this->x - a*this->y) - a*c) / (a*a + b*b);
	P.y = (a*(-b*this->x + a*this->y) - b*c) / (a*a + b*b);

	return P;
}

void np::Point::rotate( const NPFLOAT& theta ) {
	NPFLOAT xnew, ynew;
	xnew = std::cos(theta)*this->x - std::sin(theta)*this->y;
	ynew = std::sin(theta)*this->x + std::cos(theta)*this->y;

	this->x = xnew;
	this->y = ynew;
}

int np::Point::in( const Contour& C ) const {
	int result = 0;
	/* Number of vertices */
	size_t Nv = C.size();

	/* Check for degenerate case */
	if (Nv < 3) {
		return 0;
	}
	/* Current and next vertices */
	np::Point cv, nv;
	cv = C[0];
	for(size_t i = 1; i <= Nv; ++i)	{
		nv = (i == Nv ? C[0] : C[i]);
		if (nv.y == this->y)	  {
			if ((nv.x == this->x) || (cv.y == this->y &&
				 ((nv.x > this->x) == (cv.x < this->x)))) {
				return -1;
			}
		}
		if ((cv.y < this->y) != (nv.y < this->y)) {
			if (cv.x >= this->x) {
				if (nv.x > this->x) {
					result = 1 - result;
				} else {
					NPFLOAT d = (cv.x - this->x) * (nv.y - this->y) -
					(nv.x - this->x) * (cv.y - this->y);
					if (!d) {
						return -1;
					}
					if ((d > 0) == (nv.y > cv.y)) {
						result = 1 - result;
					}
				}
			} else {
				if (nv.x > this->x) {
					NPFLOAT d = (cv.x - this->x) * (nv.y - this->y) -
					(nv.x - this->x) * (cv.y - this->y);
					if (!d) {
						return -1;
					}
					if ((d > 0) == (nv.y > cv.y)) {
						result = 1 - result;
					}
				}
			}
		}
		cv = nv;
	}
	return result;
}

int np::Point::on( const Contour& C ) const {
	int result = 0;
	/* Number of vertices */
	size_t Nv = C.size();

	/* Check for degenerate case */
	if (Nv < 3) {
		return 0;
	}
	/* Current and next vertices */
	np::Point cv, nv;
	cv = C[0];
	for(size_t i = 1; i <= Nv; ++i)	{
		nv = (i == Nv ? C[0] : C[i]);
		if (nv.y == this->y) {
			if ((nv.x == this->x) || (cv.y == this->y &&
				 ((nv.x > this->x) == (cv.x < this->x)))) {
				return 1;
			}
		}
		if ((cv.y < this->y) != (nv.y < this->y)) {
			if (cv.x >= this->x) {
				if (nv.x > this->x) {
					result = 1 - result;
				} else {
					NPFLOAT d = (cv.x - this->x) * (nv.y - this->y) -
					(nv.x - this->x) * (cv.y - this->y);
					if (!d) {
						return 1;
					}
					if ((d > 0) == (nv.y > cv.y)) {
						result = 1 - result;
					}
				}
			} else {
				if (nv.x > this->x) {
					NPFLOAT d = (cv.x - this->x) * (nv.y - this->y) -
					(nv.x - this->x) * (cv.y - this->y);
					if (!d) {
						return 1;
					}
					if ((d > 0) == (nv.y > cv.y)) {
						result = 1 - result;
					}
				}
			}
		}
		cv = nv;
	}
	return std::abs(result);
}

int np::Point::in( const Polygon& P ) const {
	int inside;
	size_t Nc = P.contour.size();
	/* Loop over all contours */
	for (size_t i=0; i<Nc; i++) {
		/* No need to check open contours */
		if (!P.is_open[i]) {
			inside = this->in( P.contour[i] );
			/* Point is on a contour */
			if (inside < 0) {
				return -1;
			/* Point is inside a hole */
			} else if (inside && P.is_hole[i]) {
				return 0;
			/* Point is outside an external contour */
			} else if (!inside && !P.is_hole[i]) {
				return 0;
			}
		}
	}
	/* If the for exited normaly, the point is inside the polygon */
	return 1;
}

int np::Point::on( const Polygon& P ) const {
	int inside;
	size_t Nc = P.contour.size();
	/* Loop over all contours */
	for (size_t i=0; i<Nc; i++) {
		/* No need to check open contours */
		if (!P.is_open[i]) {
			inside = this->in( P.contour[i] );
			/* Point is on a contour */
			if (inside < 0) {
				return 1;
			/* Point is inside a hole */
			} else if (inside && P.is_hole[i]) {
				return 0;
			/* Point is outside an external contour */
			} else if (!inside && !P.is_hole[i]) {
				return 0;
			}
		}
	}
	/* If the for exited normaly, the point is inside the polygon */
	return 0;
}

int np::Point::in( const Circle& C) const {
	NPFLOAT d = np::dist(*this, C.center);

	if (std::abs(d-C.radius) <= NP_CMP_ERR) {
		return -1;
	} else if (d < C.radius) {
		return 1;
	} else {
		return 0;
	}
}

int np::Point::on( const Circle& C) const {
	NPFLOAT d = np::dist(*this, C.center);

	if (std::abs(d-C.radius) <= NP_CMP_ERR) {
		return 1;
	} else {
		return 0;
	}
}

int np::Point::on_dist( const Contour& C ) const {
	NPFLOAT d = this->dist(C);

	if (std::abs(d) <= NP_CMP_ERR) {
		return 1;
	} else {
		return 0;
	}
}

int np::Point::on_dist( const Polygon& P ) const {
	/* Loop over all contours */
	for (size_t i=0; i<P.contour.size(); i++) {
		NPFLOAT d = this->dist(P.contour[i]);
		if (std::abs(d) <= NP_CMP_ERR) {
			return 1;
		}
	}

	return 0;
}


/*----- Overloaded Operators -----*/
bool np::Point::operator == ( const Point& A ) {
	if ( this->dist(A) <= NP_CMP_ERR ) {
		return true;
	} else {
		return false;
	}
}

bool np::Point::operator != ( const Point& A ) {
	if ( this->dist(A) <= NP_CMP_ERR ) {
		return false;
	} else {
		return true;
	}
}

np::Point np::Point::operator + ( const Point& A ) {
	Point P;
	P.x = this->x + A.x;
	P.y = this->y + A.y;
	#if NP_3D_POINTS
		P.z = this->z + A.z;
	#endif

	return P;
}

np::Point np::Point::operator - ( const Point& A ) {
	Point P;
	P.x = this->x - A.x;
	P.y = this->y - A.y;
	#if NP_3D_POINTS
		P.z = this->z - A.z;
	#endif

	return P;
}

np::Point np::Point::operator - () const {
	#if NP_3D_POINTS
		return Point(-x, -y, -z);
	#else
		return Point(-x, -y);
	#endif
}

void np::Point::operator += ( const Point& A ) {
	this->x += A.x;
	this->y += A.y;
	#if NP_3D_POINTS
		this->z += A.z;
	#endif
}

void np::Point::operator -= ( const Point& A ) {
	this->x -= A.x;
	this->y -= A.y;
	#if NP_3D_POINTS
		this->z -= A.z;
	#endif
}


/*----- Non Members -----*/
NPFLOAT np::norm( const np::Point& A ) {
	#if NP_3D_POINTS
		return sqrt( A.x*A.x + A.y*A.y + A.z*A.z );
	#else
		return sqrt( A.x*A.x + A.y*A.y );
	#endif
}

NPFLOAT np::dist( const np::Point& A, const np::Point& B ) {
	#if NP_3D_POINTS
		return sqrt( pow(A.x-B.x, 2) + pow(A.y-B.y, 2) + pow(A.z-B.z, 2) );
	#else
		return sqrt( pow(A.x-B.x, 2) + pow(A.y-B.y, 2) );
	#endif
}

NPFLOAT np::dot( const np::Point& A, const np::Point& B ) {
	#if NP_3D_POINTS
		return A.x*B.x + A.y*B.y + A.z*B.z;
	#else
		return A.x*B.x + A.y*B.y;
	#endif
}

np::Point np::midpoint( const np::Point& A, const np::Point& B ) {
	#if NP_3D_POINTS
		return np::Point( (A.x+B.x)/2, (A.y+B.y)/2, (A.z+B.z)/2 );
	#else
		return np::Point( (A.x+B.x)/2, (A.y+B.y)/2 );
	#endif
}






/*********************************************************/
/********************* Contour class *********************/
/*********************************************************/
void np::Contour::read( const char* fname ) {
	FILE *fp;
	size_t  num_vertices;
	NPFLOAT x, y;

	fp = fopen(fname, "r");
	if (fp != NULL) {

		/* Read vertex number */
		fscanf(fp, "%lu", &num_vertices);
		/* Resize vectors */
		this->resize(num_vertices);

		/* Loop over each vertex */
		for (size_t i=0; i<num_vertices; i++) {

			/* Read each vertex */
			fscanf(fp, "%lf %lf", &x, &y);
			this->at(i).x = x;
			this->at(i).y = y;
		}

		fclose(fp);
	} else {
		std::cout << "Contour read error: file could not be opened" << std::endl;
	}
}

void np::Contour::write( const char* fname, const char* mode ) {
	FILE *fp;

	fp = fopen(fname, mode);
	if (fp != NULL) {

		/* Write vertex number */
		fprintf(fp, "%lu\n", this->size());

		/* Loop over each vertex */
		for (size_t i=0; i<this->size(); i++) {

			/* Write each vertex */
			fprintf(fp, "% lf % lf\n",
			(double) this->at(i).x,	(double) this->at(i).y);
		}

		fclose(fp);
	} else {
		std::cout << "Polygon write error: file could not be opened" << std::endl;
	}
}

void np::Contour::print() const {

	std::printf("Vertices %lu\n", this->size());
	for (size_t i=0; i<this->size(); i++) {
		std::printf("% .5lf % .5lf\n",
			(double) this->at(i).x, (double) this->at(i).y);
	}
}

NPFLOAT np::Contour::area() const {
	return std::abs( this->area_signed() );
}

np::Point np::Contour::centroid() const {
	np::Point C;
	size_t Nv = this->size();

	for (size_t i=0; i<Nv; i++) {
		size_t ii = (i+1) % Nv;
		C.x += (this->at(i).x + this->at(ii).x) *
			(this->at(i).x*this->at(ii).y - this->at(ii).x*this->at(i).y);
		C.y += (this->at(i).y + this->at(ii).y) *
			(this->at(i).x*this->at(ii).y - this->at(ii).x*this->at(i).y);
	}


	NPFLOAT A = this->area_signed();
	return 1/(6*A) * C;
}

void np::Contour::reverse_order() {
	np::Point tmp;
	size_t Nv = this->size();

	/* Loop over the first half vertices and swap them with the second half */
	for (size_t i=0; i<std::floor( Nv/2 ); i++) {
		tmp = this->at(i);
		this->at(i) = this->at(Nv-1-i);
		this->at(Nv-1-i) = tmp;
	}
}

bool np::Contour::is_CW() const {
	if (this->area_signed() < 0) {
		return true;
	} else {
		return false;
	}
}

void np::Contour::make_CW() {
	if (!this->is_CW()) {
		this->reverse_order();
	}
}

void np::Contour::make_CCW() {
	if (this->is_CW()) {
		this->reverse_order();
	}
}


/*----- Private Members -----*/
NPFLOAT np::Contour::area_signed() const {
	NPFLOAT A = 0;
	size_t Nv = this->size();

	for (size_t i=0; i<Nv; i++) {
		size_t ii = (i+1) % Nv;
		A += this->at(i).x * this->at(ii).y - this->at(ii).x * this->at(i).y;
	}

	return 0.5 * A;
}





/********************************************************/
/********************* Circle class *********************/
/********************************************************/
np::Circle::Circle() {
	this->center = Point();
	this->radius = 0;
}

np::Circle::Circle( const np::Point& P, const NPFLOAT& R ) {
	this->center.x = P.x;
	this->center.y = P.y;
	this->radius = R;
}

np::Circle::Circle( const np::Point& P ) {
	this->center.x = P.x;
	this->center.y = P.y;
	this->radius = 0;
}


NPFLOAT np::Circle::area() const {
	return M_PI * this->radius * this->radius;
}

bool np::Circle::is_point() const {
	if (this->radius == 0) {
		return true;
	} else {
		return false;
	}
}







/*********************************************************/
/********************* Polygon class *********************/
/*********************************************************/
np::Polygon::Polygon( ) {}

np::Polygon::Polygon( const char* filename ) {
	this->np::Polygon::read( filename );
}

np::Polygon::Polygon( const np::Point& P ) {
	this->contour.resize(1);
	this->is_hole.resize(1);
	this->is_open.resize(1);
	this->contour[0].push_back(P);
	this->is_hole[0] = false;
	this->is_open[0] = true;
}

np::Polygon::Polygon( const np::Circle& C, size_t points_per_circle ) {
	this->contour.resize(1);
	this->is_hole.resize(1);
	this->is_open.resize(1);
	this->contour[0].resize(points_per_circle);
	this->is_hole[0] = false;
	this->is_open[0] = false;

	NPFLOAT dt = 2*M_PI / (points_per_circle);

	for (size_t i=0; i<points_per_circle; i++) {
		this->contour[0][i].x = C.radius * std::cos( i*dt ) + C.center.x;
		this->contour[0][i].y = C.radius * std::sin( i*dt ) + C.center.y;
	}
}


void np::Polygon::read( const char* fname, bool read_hole, bool read_open ) {
	FILE *fp;
	size_t num_contours, num_vertices;
	int is_hole, is_open;
	NPFLOAT x, y;

	fp = fopen(fname, "r");
	if (fp != NULL) {

		/* Read contour number */
		fscanf(fp, "%lu", &num_contours);
		/* Resize vectors */
		this->contour.resize(num_contours);
		this->is_hole.resize(num_contours);
		this->is_open.resize(num_contours);

		/* Loop over each contour */
		for (size_t i=0; i<num_contours; i++) {

			/* Read vertex number */
			fscanf(fp, "%lu", &num_vertices);
			this->contour[i].resize(num_vertices);

			/* Read hole flag */
			if (read_hole) {
				fscanf(fp, "%d", &is_hole);
				this->is_hole[i] = is_hole;
			} else {
				this->is_hole[i] = false;
			}

			/* Read open flag */
			if (read_open) {
				fscanf(fp, "%d", &is_open);
				this->is_open[i] = is_open;
			} else {
				this->is_open[i] = false;
			}

			/* Loop over each vertex */
			for (size_t j=0; j<num_vertices; j++) {

				/* Read each vertex */
				fscanf(fp, "%lf %lf", &x, &y);
				this->contour[i][j].x = x;
				this->contour[i][j].y = y;
			}

			/* Set contour orientation */
			if (this->is_hole[i]) {
				this->contour[i].make_CCW();
			} else {
				this->contour[i].make_CW();
			}
		}

		fclose(fp);
	} else {
		std::cout << "Polygon read error: file could not be opened" << std::endl;
	}
}

void np::Polygon::write( const char* fname, bool write_hole, bool write_open, const char* mode ) {
	FILE *fp;

	fp = fopen(fname, mode);
	if (fp != NULL) {

		/* Write contour number */
		fprintf(fp, "%lu\n", this->contour.size());

		/* Loop over each contour */
		for (size_t i=0; i<this->contour.size(); i++) {

			/* Write vertex number */
			fprintf(fp, "%lu\n", this->contour[i].size());

			/* Write hole flag */
			if (write_hole) {
				fprintf(fp, "%d\n", (int) this->is_hole[i]);
			}

			/* Write open flag */
			if (write_open) {
				fprintf(fp, "%d\n", (int) this->is_open[i]);
			}

			/* Loop over each vertex */
			for (size_t j=0; j<this->contour[i].size(); j++) {

				/* Write each vertex */
				fprintf(fp, "% lf % lf\n",
				(double) this->contour[i][j].x,	(double) this->contour[i][j].y);
			}
		}

		fclose(fp);
	} else {
		std::cout << "Polygon write error: file could not be opened" << std::endl;
	}
}

void np::Polygon::print() const {

	std::printf("Contours: %lu\n", this->contour.size());
	for (size_t i=0; i<this->contour.size(); i++) {
		std::printf("Contour %lu: Hole %d, Open %d, Vertices %lu\n",
			i, (int) this->is_hole[i], (int) this->is_open[i], this->contour[i].size());

		for (size_t j=0; j<this->contour[i].size(); j++) {
			std::printf("% .20lf % .20lf\n",
				(double) this->contour[i][j].x, (double) this->contour[i][j].y);
		}
	}
}


NPFLOAT np::Polygon::diameter() const {
	/* TO DO: use convex hull to reduce complexity */
	NPFLOAT d = 0;
	np::Contour verts = this->get_vertices();

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

NPFLOAT np::Polygon::area() const {
	/* TO DO: How does it handle non-simple polygons? */
	NPFLOAT A = 0;

	/* Loop over all contours */
	for (size_t i=0; i<this->contour.size(); i++) {
		/* If the contour is a hole, subtract its area */
		if (this->is_hole[i] && !this->is_open[i]) {
			A -= this->contour[i].area();
		/* If the contour is not a hole, add its area */
		} else if (!this->is_hole[i] && !this->is_open[i]) {
			A += this->contour[i].area();
		}
	}

	return A;
}

np::Point np::Polygon::centroid() const {
	/* TO DO: How does it handle non-simple polygons? */
	/* https://math.stackexchange.com/questions/623841/finding-centroid-of-polygon-with-holes-polygons */

	np::Point C;

	/* Loop over all contours */
	for (size_t i=0; i<this->contour.size(); i++) {
		/* If the contour is a hole, subtract its weighted centroid */
		if (this->is_hole[i] && !this->is_open[i]) {
			C -= this->contour[i].area() * this->contour[i].centroid();
		/* If the contour is not a hole, add its weighted centroid */
		} else if (!this->is_hole[i] && !this->is_open[i]) {
			C += this->contour[i].area() * this->contour[i].centroid();
		}
	}

	return C / this->area();
}

np::Point np::Polygon::normal(size_t& c, size_t& e) const {
	/* Get the two edge vertices */
	np::Point v1, v2, n;
	v1 = this->contour[c][e];
	v2 = this->contour[c][(e+1) % this->contour[c].size()];
	n = v2 - v1;

	/* Assume external contours are CW and internal ones CCW */
	/* Check the orientation of the contour and rotate accordingly */
	if (this->contour[c].is_CW()) {
		/* External contour, rotate 90 degrees */
		n.rotate( std::asin(1) );
	} else {
		/* Internal contour, rotate -90 degrees */
		n.rotate( -std::asin(1) );
	}

	/* Make into a unit vector */
	n = n / n.norm();

	return n;
}


bool np::Polygon::correct_orientation() const {
	for (size_t i=0; i<this->contour.size(); i++) {
		if (this->is_hole[i]) {
			if (this->contour[i].is_CW()) {
				/* If an internal contour is CW */
				return false;
			}
		} else {
			if (!this->contour[i].is_CW()) {
				/* If an external contour is CCW */
				return false;
			}
		}
	}
	return true;
}

bool np::Polygon::is_point() const {
	if ( (this->contour.size() == 1) && (this->contour[0].size() == 1) ) {
		return true;
	} else {
		return false;
	}
}

bool np::Polygon::is_empty() const {
	if (this->contour.size() == 0) {
		return true;
	} else {
		return false;
	}
}

void np::Polygon::make_empty() {
	this->contour.resize(0);
	this->is_hole.resize(0);
	this->is_open.resize(0);
}

void np::Polygon::fix_orientation( bool follow_hole_flags ) {
	if (follow_hole_flags) {
		/* Set orientation according to hole flags */
		for (size_t i=0; i<this->contour.size(); i++) {
			if (this->is_hole[i]) {
				if (this->contour[i].is_CW()) {
					/* If an internal contour is CW */
					this->contour[i].reverse_order();
				}
			} else {
				if (!this->contour[i].is_CW()) {
					/* If an external contour is CCW */
					this->contour[i].reverse_order();
				}
			}
		}
	} else {
		/* Set orientation according to contour vertex order */
		for (size_t i=0; i<this->contour.size(); i++) {
			if (this->contour[i].is_CW()) {
				/* Set as external contour */
				this->is_hole[i] = false;
			} else {
				/* Set as interal contour */
				this->is_hole[i] = true;
			}
		}
	}
}


void np::Polygon::translate( const Point& P ) {
	/* Loop over all contours */
	for (size_t i=0; i<this->contour.size(); i++) {
		/* Loop over all vertices */
		for (size_t j=0; j<this->contour[i].size(); j++) {
			/* Translate the vertex */
			this->contour[i].at(j) += P;
		}
	}
}

void np::Polygon::rotate( const NPFLOAT& theta, const bool around_origin ) {
	np::Point C = this->centroid();

	if (!around_origin) {
		/* Translate the polygon so that its centroid is on the origin */
		this->translate(-C);
	}

	/* Loop over all contours */
	for (size_t i=0; i<this->contour.size(); i++) {
		/* Loop over all vertices */
		for (size_t j=0; j<this->contour[i].size(); j++) {
			/* Rotate the vertex */
			this->contour[i][j].rotate(theta);
		}
	}

	if (!around_origin) {
		/* Translate the polygon to its original position */
		this->translate(C);
	}
}


bool np::Polygon::contains( const Point& ) const {
	return false;
}

np::Polygon np::Polygon::convex_hull() const {
	/* https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain */
	return np::Polygon();
}


/*----- Private Members -----*/
np::Contour np::Polygon::get_vertices() const {
	/* Find the total number of vertices */
	size_t Nv = 0;
	for (size_t i=0; i<this->contour.size(); i++) {
		Nv += this->contour[i].size();
	}

	/* Create destination vector */
	np::Contour verts;
	verts.resize(Nv);

	/* Copy vertices to the vector */
	size_t k = 0;
	for (size_t i=0; i<this->contour.size(); i++) {
		for (size_t j=0; j<this->contour[i].size(); j++) {
			verts.at(k++) = this->contour[i].at(j);
		}
	}

	return verts;
}


/*----- Non Members -----*/
void np::operator + ( np::Polygon& P, const np::Point& A ) {
	P.translate(A);
}

void np::operator + ( const np::Point& A, np::Polygon& P ) {
	P.translate(A);
}

void np::operator - ( np::Polygon& P, const np::Point& A ) {
	P.translate(-A);
}







/**********************************************************/
/********************* Polygons class *********************/
/**********************************************************/
np::Polygons::Polygons() {}

np::Polygons::Polygons( const np::Circles& C, size_t points_per_circle ) {
	this->resize(C.size());
	for (size_t i=0; i<C.size(); i++) {
		this->at(i) = np::Polygon( C[i], points_per_circle );
	}
}


void np::Polygons::print() const {

	std::printf("Polygons: %lu\n", this->size());
	for (size_t k=0; k<this->size(); k++) {
		std::printf("Polygon %lu Contours: %lu\n", k, this->at(k).contour.size());
		for (size_t i=0; i<this->at(k).contour.size(); i++) {
			std::printf("Contour %lu: Hole %d, Open %d\n",
				i, (int) this->at(k).is_hole[i], (int) this->at(k).is_open[i]);

			for (size_t j=0; j<this->at(k).contour[i].size(); j++) {
				std::printf("% .5lf % .5lf\n",
					(double) this->at(k).contour[i][j].x, (double) this->at(k).contour[i][j].y);
			}
		}
	}
}
