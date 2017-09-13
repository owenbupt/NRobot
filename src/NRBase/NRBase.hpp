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

#ifndef __NRBase_hpp
#define __NRBase_hpp

#include <iostream>
#include <vector>

#include "NRConfig.hpp"


/* Define Pi */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


namespace nr {

/* Forward class declarations */
class Point;
class Contour;
class Polygon;
class Polygons;
class Circle;
class Circles;
class Ellipse;
class Orientation;
/* Typedef for ease of use */
typedef Contour Points;
typedef std::vector<Orientation> Orientations;




/*******************************************************/
/********************* Point class *********************/
/*******************************************************/
/*!
	Single 3D point class.
*/
class Point {
	public:
		/****** Data members ******/
		double x;
		double y;
		double z;

		/****** Constructor ******/
		/* Default behavior is a point at the origin */
		Point( const double x = 0, const double y = 0, const double z = 0 );
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
};




/*********************************************************/
/********************* Polygon class *********************/
/*********************************************************/
/*!
	Signle 2D polygon. Can contain both internal and external contours.
*/
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
		Polygon( const Point& P );
		Polygon( const Contour& C );
		Polygon( const Circle& C, size_t points_per_circle = NR_PPC );
		Polygon( const Ellipse& E, size_t points_per_circle = NR_PPC );
};




/**********************************************************/
/********************* Polygons class *********************/
/**********************************************************/
/*!
	Vector of 2D polygons.
*/
class Polygons: public std::vector<Polygon> {
	public:
		/****** Constructor ******/
		Polygons();
		Polygons( const Circles& C, size_t points_per_circle = NR_PPC );
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
		Circle( const Point& center = Point(0,0), double r = 0 );
};




/*********************************************************/
/********************* Circles class *********************/
/*********************************************************/
class Circles: public std::vector<Circle> {
	public:
		/****** Constructor ******/
		Circles();
		Circles( const Points& centers, const std::vector<double>& radii );
};




/*********************************************************/
/********************* Ellipse class *********************/
/*********************************************************/
class Ellipse {
	public:
		/****** Data members ******/
		Point center;
		Point focus1;
		Point focus2;
		double a;
		double b;
		double c;
		double theta;
		double eccentricity;

		/****** Constructor ******/
		Ellipse(
			double a = 2,
			double b = 1,
			const Point& center = Point(0,0),
			double theta = 0
		);
};




/*******************************************************/
/******************* Attitude class ********************/
/*******************************************************/
/*!
	3D attitude class. Angles in radians. Domains for roll and yaw are [0, 2pi]
	and for pitch [-pi/2, pi/2].
*/
class Orientation {
	public:
		/****** Data members ******/
		double roll;
		double pitch;
		double yaw;

		/****** Constructor ******/
		/* Default behavior is all angles being zero */
		Orientation( double r = 0, double p = 0, double y = 0 );
};







/*********************************************************/
/********************* Non Members **********************/
/*********************************************************/

/****** Operator overloads ******/
bool operator == ( const Point&, const Point& );
bool operator != ( const Point&, const Point& );
Point operator + ( const Point&, const Point& );
Point operator - ( const Point&, const Point& );
Point operator - ( const Point& );
void operator += ( Point&, const Point& );
void operator -= ( Point&, const Point& );
bool operator == ( const Orientation&, const Orientation& );
bool operator != ( const Orientation&, const Orientation& );
Orientation operator + ( const Orientation&, const Orientation& );
Orientation operator - ( const Orientation&, const Orientation& );
Orientation operator - ( const Orientation& );
void operator += ( Orientation&, const Orientation& );
void operator -= ( Orientation&, const Orientation& );
void operator + ( Polygon&, const Point& );
void operator + ( const Point&, Polygon& );
void operator - ( Polygon&, const Point& );
/* Scalar multiplication and division oveloading */
template <class T> Point operator * ( const Point& P, T k ) {
	return Point(k*P.x, k*P.y, k*P.z);
}
template <class T> Point operator * ( T k, const Point& P ) {
	return Point(k*P.x, k*P.y, k*P.z);
}
template <class T> Point operator / ( const Point& P, T k ) {
	return Point(P.x/k, P.y/k, P.z/k);
}
/* Stream operator overloading */
std::ostream& operator << ( std::ostream& output, const Point& P );
std::ostream& operator << ( std::ostream& output, const Contour& C );

/****** Point ******/
void print( const Point& A );
double norm( const Point& A );
double dist( const Point& A, const Point& B );
double dist( const Point& A, const Contour& C ); 					/* TODO */
double dist( const Point& A, const Polygon& P ); 					/* TODO */
double dot( const Point& A, const Point& B );
Point rotate( const Point& A, double theta );
Point midpoint( const Point& A, const Point& B );
// double dist_from_line( const Point& A, const Point&, const Point& );
// Point closest_to_line( const Point& A, const Point&, const Point& );
bool in( const Point& A, const Contour& C );
bool on( const Point& A, const Contour& C );
bool in( const Point& A, const Polygon& P );
bool on( const Point& A, const Polygon& P );
// bool in( const Point& A, const Circle& C );
// bool on( const Point& A, const Circle& C );
bool is_vertex_of( const Point& A, const Polygon& P );

/****** Contour ******/
int read( Contour* C, const char* fname );
int write( const Contour& C, const char* fname, const char* mode = "w" );
void print( const Contour& C );
double area( const Contour& C, bool signed_area = false );
Point centroid( const Contour& C );
bool is_CW( const Contour& C );
void reverse_order( Contour* C );
void make_CW( Contour* C );
void make_CCW( Contour* C );

/****** Polygon ******/
int read( Polygon* P, const char* fname, bool read_hole = false, bool read_open = false );
int write( const Polygon& P, const char* fname, bool write_hole = false, bool write_open = false, const char* mode = "w" );
void print( const Polygon& P );
double diameter( const Polygon& P );
double radius( const Polygon& P, const Point& reference_point = Point(0,0) );
double area( const Polygon& P );
Point centroid( const Polygon& P );
Point normal( const Polygon& P, size_t contour, size_t edge );
bool is_orientation_correct( const Polygon& P );
bool is_point( const Polygon& P );
bool is_empty( const Polygon& P );
void make_empty( Polygon* P );
void fix_orientation( Polygon* P, bool follow_hole_flags = true );
void translate( Polygon* P, const Point& p );
void rotate(
	Polygon* P,
	double theta,
	bool around_origin = false
);
/* Rotate P theta radians around its centroid, unless the around_origin flag is true */
void scale( Polygon* P, double scale_factor );
Contour convex_hull( const Polygon& P ); 							/* TODO */

/****** Polygons ******/
void print( const Polygons& P );

/****** Circle ******/
double area( const Circle& C );
bool is_point( const Circle& C );

/****** Orientation ******/
void print( const Orientation& A );

/****** Others ******/
Point projection(
	const Point& A,
	const Point& B
);
/* Returns the projection of vector A on vector B. The formula used is
   p = (a * b) / (b * b) b, where * denotes the vector dot product. */

std::vector<double> linspace(
	double start,
	double end,
	size_t num = 100
);
/* Creates a linearly spaced vector from start to end inclusive with num elements */

Point cart2pol(
	const Point& P
);
/* Convert a point from cartesian to polar coordinates */

Point pol2cart(
	const Point& P
);
/* Convert a point from polar to cartesian coordinates */


/*******************************************************/
/*********************** enums *************************/
/*******************************************************/
enum error_type {
	SUCCESS,
	ERROR_INVALID_SUBJECT,
	ERROR_INVALID_CLIP,
	ERROR_CLIPPING_FAILED,
	ERROR_PARTITIONING_FAILED,
	ERROR_INVALID_PARTITIONING
};

} /* End of namespace */

#endif
