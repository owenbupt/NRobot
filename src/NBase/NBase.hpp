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
    along with NPart.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __NBase_hpp
#define __NBase_hpp

#include <iostream>
#include <vector>

#include "NConfig.hpp"


/* Define Pi */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


namespace n {

/* Forward class declarations */
class Point;
class Contour;
class Polygon;
class Polygons;
class Circle;
class Circles;
/* Typedef for ease of use */
typedef Contour Points;




/*******************************************************/
/********************* Point class *********************/
/*******************************************************/
/*!
	Single 3D point class and overloaded operators.
*/
class Point {
	public:
		/****** Data members ******/
		double x;
		double y;
		double z;

		/****** Constructor ******/
		/* Default behavior is a point at the origin */
		Point( double x = 0, double y = 0, double z = 0 );

		/****** Overloaded Operators ******/
		/* Equality, addition and subtraction overloading */
		bool operator == ( const Point& );
		bool operator != ( const Point& );
		Point operator + ( const Point& );
		Point operator - ( const Point& );
		Point operator - () const;
		void operator += ( const Point& );
		void operator -= ( const Point& );

		/* Scalar multiplication and division oveloading */
		template <class T> friend Point operator * ( const Point& P, T k ) {
			return Point(k*P.x, k*P.y, k*P.z);
		}
		template <class T> friend Point operator * ( T k, const Point& P ) {
			return Point(k*P.x, k*P.y, k*P.z);
		}
		template <class T> friend Point operator / ( const Point& P, T k ) {
			return Point(P.x/k, P.y/k, P.z/k);
		}

		/* Stream output operator */
		friend std::ostream& operator << ( std::ostream& output, const Point& P ) {
			output << P.x << " " << P.y << " " << P.z;
			return output;
		}
};




/*********************************************************/
/********************* Contour class *********************/
/*********************************************************/
/*!
	Vector of 3D points. Can define a polyline or polygon contour.
*/
class Contour: public std::vector<Point> {
	public:
		/****** Constructor ******/
		/* Uses the default constructor for the vector and its elements */

		/****** Overloaded Operators ******/
		/* Stream output operator */
		friend std::ostream& operator << ( std::ostream& output, const Contour& C ) {
			for (size_t i=0; i<C.size(); i++) {
				output << C.at(i) << "\n";
			}
			return output;
		}
};




/*********************************************************/
/********************* Polygon class *********************/
/*********************************************************/
class Polygon {
	public:
		/****** Data members ******/
		std::vector<Contour> contour;
		std::vector<bool> is_hole;
		std::vector<bool> is_open;
		/* External contours are CW, internal ones CCW */

		/****** Constructor ******/
		/* Default behavior is an empty polygon, no contours-vertices */
		Polygon();
};




/**********************************************************/
/********************* Polygons class *********************/
/**********************************************************/
class Polygons: public std::vector<Polygon> {
	public:
		/****** Constructor ******/
		/* Uses the default constructor for the vector and its elements */
};




/********************************************************/
/********************* Circle class *********************/
/********************************************************/
class Circle {
	public:
		/****** Data members ******/
		Point center;
		double radius;

		/****** Constructor ******/
		/* Default behavior is center at origin and zero radius */
		Circle();
		Circle( Point& C, double r = 0 );
};




/*********************************************************/
/********************* Circles class *********************/
/*********************************************************/
class Circles: public std::vector<Circle> {
	public:
		/****** Constructor ******/
		/* Uses the default constructor for the vector and its elements */
		Circles();
		Circles( Points& centers, std::vector<double>& radii );
};








/*********************************************************/
/********************* Non Members **********************/
/*********************************************************/

/* Point */
double norm( Point& A );
double dist( Point& A, Point& B );
double dist( Point& A, Contour& C ); /* TODO */
double dist( Point& A, Polygon& P ); /* TODO */
double dot( Point& A, Point& B );
n::Point rotate( Point& A, double theta );
n::Point midpoint( Point& A, Point& B );
// double dist_from_line( Point& A, Point&, Point& );
// Point closest_to_line( Point& A, Point&, Point& );
// bool in( Point& A, Contour& C );
// bool on( Point& A, Contour& C );
// bool in( Point& A, Polygon& P );
// bool on( Point& A, Polygon& P );
// bool in( Point& A, Circle& C );
// bool on( Point& A, Circle& C );
// bool on_dist( Point& A, Contour& C );
// bool on_dist( Point& A, Polygon& P );

/* Contour */
int read( Contour& C, const char* fname );
int write( Contour& C, const char* fname, const char* mode = "w" );
void print( Contour& C );
double area( Contour& C, bool signed_area = false );
n::Point centroid( Contour& C );
bool is_CW( Contour& C );
void reverse_order( Contour* C );
void make_CW( Contour* C );
void make_CCW( Contour* C );

/* Polygon */
// Polygon( const Point& );
// Polygon( const Circle&, size_t points_per_circle = NP_PPC );
// /* C-style read/write/print */
// Polygon( const char* filename );
// void read( const char* fname, bool read_hole = false, bool read_open = false );
// void write( const char* fname, bool write_hole = false, bool write_open = false, const char* mode = "w" );
// void print() const;
//
// double diameter() const;
// double area() const;
// Point centroid() const;
// Point normal(size_t& c, size_t& e) const;
// /* Returns the outwards unit normal vector at edge e of contour c */
//
// bool correct_orientation() const;
// bool is_point() const;
// bool is_empty() const;
// void make_empty();
// void fix_orientation( bool follow_hole_flags = true );
//
//
// void operator + ( Polygon&, const Point& );
// void operator + ( const Point&, Polygon& );
// void operator - ( Polygon&, const Point& );

/* Polygons */
// Polygons( const Circles&, size_t points_per_circle = NP_PPC );
// void print() const;

/* Circle */
double area( Circle& C );
bool is_point( Circle& C );

/* Private */
// Contour get_vertices( Polygon ) const;

} /* End of namespace */

#endif
