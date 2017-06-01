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


#ifndef __NPBase_h
#define __NPBase_h

#include <iostream>
#include <vector>
#include "NPConfig.hpp"


/* Define Pi */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



namespace np {

/* Forward class declarations */
class Point;
class Contour;
class Polygon;
class Polygons;
class Circle;
class Circles;

/*******************************************************/
/********************* Point class *********************/
/*******************************************************/
/*!
	A class representing a 2D point.
*/
class Point {
	public:
		/*----- Public Members -----*/
		NPFLOAT x;
		NPFLOAT y;
		/* Define z coordinate if desired */
		#if NP_3D_POINTS
			NPFLOAT z;
		#endif

		Point();
		Point( NPFLOAT x, NPFLOAT y );
		#if NP_3D_POINTS
			Point( NPFLOAT X, NPFLOAT Y, NPFLOAT Z );
		#endif

		NPFLOAT norm() const;
		NPFLOAT dist( const Point& ) const;
		NPFLOAT dist( const Contour& ) const;
		NPFLOAT dist( const Polygon& ) const;
		NPFLOAT dist_from_line( const Point&, const Point& ) const;
		NPFLOAT dot( const Point& ) const;
		Point midpoint( const Point& ) const;
		Point closest_to_line( const Point&, const Point& ) const;
		void rotate( const NPFLOAT& theta );
		int in( const Contour& ) const;
		int on( const Contour& ) const;
		int in( const Polygon& ) const;
		int on( const Polygon& ) const;
		int in( const Circle& ) const;
		int on( const Circle& ) const;
		int on_dist( const Contour& ) const;
		int on_dist( const Polygon& ) const;


		/*----- Overloaded Operators -----*/
		/* Equality, Addition and subtraction overloading */
		bool operator == ( const Point& );
		bool operator != ( const Point& );
		Point operator + ( const Point& );
		Point operator - ( const Point& );
		Point operator - () const;
		void operator += ( const Point& );
		void operator -= ( const Point& );

		/* Scalar multiplication and division oveloading */
		template <class T> friend Point operator * ( const Point& P, T k ) {
			#if NP_3D_POINTS
				return Point(k*P.x, k*P.y, k*P.z);
			#else
				return Point(k*P.x, k*P.y);
			#endif
		}
		template <class T> friend Point operator * ( T k, const Point& P ) {
			#if NP_3D_POINTS
				return Point(k*P.x, k*P.y, k*P.z);
			#else
				return Point(k*P.x, k*P.y);
			#endif
		}
		template <class T> friend Point operator / ( const Point& P, T k ) {
			#if NP_3D_POINTS
				return Point(P.x/k, P.y/k, P.z/k);
			#else
				return Point(P.x/k, P.y/k);
			#endif
		}

		/* Stream operators */
		friend std::ostream& operator << ( std::ostream& output, const Point& P ) {
			#if NP_3D_POINTS
				output << P.x << " " << P.y << " " << P.z;
			#else
				output << P.x << " " << P.y;
			#endif
			return output;
		}

};

/*----- Non Members -----*/
NPFLOAT norm( const Point& A );
NPFLOAT dist( const Point& A, const Point& B );
NPFLOAT dot( const Point& A, const Point& B );
np::Point midpoint( const Point& A, const Point& B );





/*********************************************************/
/********************* Contour class *********************/
/*********************************************************/
/*!
	A class representing a list of 2D points. It can be used as a polygon
	contour or as a series of line segments.
*/
class Contour: public std::vector<Point> {
	public:
		/* C-style read/write/print */
		void read( const char* fname );
		void write( const char* fname, const char* mode = "w" );
		void print() const;

		NPFLOAT area() const;
		Point centroid() const;

		void reverse_order();
		bool is_CW() const;
		void make_CW();
		void make_CCW();

		/* Stream operators */
		friend std::ostream& operator << ( std::ostream& output, const Contour& C ) {
			for (size_t i=0; i<C.size(); i++) {
				output << C.at(i) << "\n";
			}
			return output;
		}

	private:
		NPFLOAT area_signed() const ;
};
typedef Contour Points;







/********************************************************/
/********************* Circle class *********************/
/********************************************************/
/*!
	A class representing a circle.
*/
class Circle {
	public:
		Point center;
		NPFLOAT radius;

		Circle();
		Circle( const Point&, const NPFLOAT& );
		Circle( const Point& );

		NPFLOAT area() const;
		bool is_point() const;
};





/*********************************************************/
/********************* Circles class *********************/
/*********************************************************/
/*!
	A class representing a list of circles.
*/
class Circles: public std::vector<Circle> {

};







/*********************************************************/
/********************* Polygon class *********************/
/*********************************************************/
/*!
	A class representing a 2D polygon. The polygon is allowed to have holes
	and be complex.
*/
class Polygon {
	// friend class Contour;
	public:
		std::vector<Contour> contour;
		std::vector<bool> is_hole;
		std::vector<bool> is_open;
		/* External contours are CW, internal ones CCW */

		Polygon();
		Polygon( const char* filename );
		Polygon( const Point& );
		Polygon( const Circle&, size_t points_per_circle = NP_PPC );

		/* C-style read/write/print */
		void read( const char* fname, bool read_hole = false, bool read_open = false );
		void write( const char* fname, bool write_hole = false, bool write_open = false, const char* mode = "w" );
		void print() const;

		NPFLOAT diameter() const;
		NPFLOAT area() const;
		Point centroid() const;
		Point normal(size_t& c, size_t& e) const;
		/* Returns the outwards unit normal vector at edge e of contour c */

		bool correct_orientation() const;
		bool is_point() const;
		bool is_empty() const;
		void make_empty();
		void fix_orientation( bool follow_hole_flags = true );

		void translate( const Point& );
		void rotate( const NPFLOAT&, const bool around_origin = false );

		bool contains( const Point& ) const;
		Polygon convex_hull() const;

	private:
		Contour get_vertices() const;
};

/*----- Non Members -----*/
void operator + ( Polygon&, const Point& );
void operator + ( const Point&, Polygon& );
void operator - ( Polygon&, const Point& );





/**********************************************************/
/********************* Polygons class *********************/
/**********************************************************/
/*!
	A class representing a list of 2D polygons.
*/
class Polygons: public std::vector<Polygon> {
	public:
		Polygons();
		Polygons( const Circles&, size_t points_per_circle = NP_PPC );

		void print() const;
};




} /* End of namespace */

#endif
