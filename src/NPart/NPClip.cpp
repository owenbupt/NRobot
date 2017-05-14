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

#include <iostream>
#include <cmath>
#include "clipper.hpp"
#include "NPClip.hpp"

#define USE_EXPONENT_SCALINGN


bool npclip::polygon_clip_fast(
	npclip::Clip_t clip_type,
	const np::Polygon& S1, const np::Polygon& S2,
	np::Polygon& R ) {

	/****** Find the appropriate scaling factor ******/
	NPFLOAT sc = std::pow(10,15);
	// int sc = 15;

	/****** Create subject Polygon paths from S1 ******/
	ClipperLib::Paths subj( S1.contour.size() );
	for (size_t i=0; i<S1.contour.size(); i++) {
		subj[i].resize( S1.contour[i].size() );
		for (size_t j=0; j<S1.contour[i].size(); j++) {
			subj[i][j].X = sc * S1.contour[i][j].x;
			subj[i][j].Y = sc * S1.contour[i][j].y;
			// subj[i][j].X = std::scalbn(S1.contour[i][j].x, sc);
			// subj[i][j].Y = std::scalbn(S1.contour[i][j].y, sc);
		}
	}

	/****** Create clip Polygon paths from S2 ******/
	ClipperLib::Paths clip( S2.contour.size() );
	for (size_t i=0; i<S2.contour.size(); i++) {
		clip[i].resize( S2.contour[i].size() );
		for (size_t j=0; j<S2.contour[i].size(); j++) {
			clip[i][j].X = sc * S2.contour[i][j].x;
			clip[i][j].Y = sc * S2.contour[i][j].y;
			// subj[i][j].X = std::scalbn(S2.contour[i][j].x, sc);
			// subj[i][j].Y = std::scalbn(S2.contour[i][j].y, sc);
		}
	}

	/****** Initialize Clipper class ******/
	ClipperLib::Clipper clpr;
	clpr.StrictlySimple(STRICTLY_SIMPLE);

	/****** Add the paths/Polygons to the clipper class ******/
	if ( !clpr.AddPaths(subj, ClipperLib::ptSubject, true) ) {
		printf("Clipper error: Invalid subject np::Polygon %p.\n", &S1);
		return false;
	}
	if ( !clpr.AddPaths(clip, ClipperLib::ptClip, true) ) {
		printf("Clipper error: Invalid clip np::Polygon %p.\n", &S2);
		return false;
	}

	/****** Set clipping operation ******/
	ClipperLib::ClipType clipType;
	switch (clip_type) {
		case AND: clipType = ClipperLib::ctIntersection; break;
		case OR: clipType = ClipperLib::ctUnion; break;
		case XOR: clipType = ClipperLib::ctXor; break;
		case DIFF: clipType = ClipperLib::ctDifference; break;
	}

	/****** Execute clipping ******/
	ClipperLib::Paths result;
	if ( !clpr.Execute( clipType, result ) ) {
		std::cout << "Clipper error: np::Polygon clipping failed.\n";
		return false;
	}

	/****** Create Polygon from paths ******/
	size_t Nc = result.size();
	R.contour.resize(Nc);
	R.is_hole.resize(Nc);
	R.is_open.resize(Nc);
	for (size_t i=0; i<Nc; i++) {
		R.is_hole[i] = false;
		R.is_open[i] = false;

		size_t Nv = result[i].size();
		R.contour[i].resize(Nv);
		for (size_t j=0; j<Nv; j++) {
			R.contour[i][j].x = result[i][j].X / sc;
			R.contour[i][j].y = result[i][j].Y / sc;
			// R.contour[i][j].x = std::scalbn(result[i][j].X, -sc);
			// R.contour[i][j].y = std::scalbn(result[i][j].Y, -sc);
		}
	}

	return true;
}


bool npclip::polygon_clip(
	npclip::Clip_t clip_type,
	const np::Polygon& S1, const np::Polygon& S2,
	np::Polygon& R ) {

	/****** Check for empty inputs ******/
	if (S1.is_empty() && S2.is_empty()) {
		/* Both polygons are empty */
		R.make_empty();
		return true;
	} else if (S1.is_empty()) {
		/* Subject polygon is empty */
		switch (clip_type) {
			case AND:
			case DIFF:
				R.make_empty();
				break;
			case OR:
			case XOR:
				R = S2;
				break;
		}
		return true;
	} else if (S2.is_empty()) {
		/* Clip polygon is empty */
		switch (clip_type) {
			case AND:
				R.make_empty();
				break;
			case OR:
			case XOR:
			case DIFF:
				R = S1;
				break;
		}
		return true;
	}
	/* Else continue normally */

	/****** Find the appropriate scaling factor ******/
	#ifndef USE_EXPONENT_SCALING
		NPFLOAT sc = std::pow(10,15);
	#else
		int sc = 15;
	#endif

	/****** Create subject Polygon paths from S1 ******/
	ClipperLib::Paths subj( S1.contour.size() );
	for (size_t i=0; i<S1.contour.size(); i++) {
		subj[i].resize( S1.contour[i].size() );
		for (size_t j=0; j<S1.contour[i].size(); j++) {
			#ifndef USE_EXPONENT_SCALING
				subj[i][j].Y = sc * S1.contour[i][j].y;
				subj[i][j].X = sc * S1.contour[i][j].x;
			#else
				subj[i][j].X = std::scalbn(S1.contour[i][j].x, sc);
				subj[i][j].Y = std::scalbn(S1.contour[i][j].y, sc);
			#endif
		}
	}

	/****** Create clip Polygon paths from S2 ******/
	ClipperLib::Paths clip( S2.contour.size() );
	for (size_t i=0; i<S2.contour.size(); i++) {
		clip[i].resize( S2.contour[i].size() );
		for (size_t j=0; j<S2.contour[i].size(); j++) {
			#ifndef USE_EXPONENT_SCALING
				clip[i][j].Y = sc * S2.contour[i][j].y;
				clip[i][j].X = sc * S2.contour[i][j].x;
			#else
				subj[i][j].X = std::scalbn(S2.contour[i][j].x, sc);
				subj[i][j].Y = std::scalbn(S2.contour[i][j].y, sc);
			#endif
		}
	}

	/****** Initialize Clipper class ******/
	ClipperLib::Clipper clpr;
	clpr.StrictlySimple(STRICTLY_SIMPLE);

	/****** Add the paths/Polygons to the clipper class ******/
	if ( !clpr.AddPaths(subj, ClipperLib::ptSubject, true) ) {
		printf("Clipper error: Invalid subject np::Polygon %p.\n", &S1);
		return false;
	}
	if ( !clpr.AddPaths(clip, ClipperLib::ptClip, true) ) {
		printf("Clipper error: Invalid clip np::Polygon %p.\n", &S2);
		return false;
	}

	/****** Set clipping operation ******/
	ClipperLib::ClipType clipType;
	switch (clip_type) {
		case AND: clipType = ClipperLib::ctIntersection; break;
		case OR: clipType = ClipperLib::ctUnion; break;
		case XOR: clipType = ClipperLib::ctXor; break;
		case DIFF: clipType = ClipperLib::ctDifference; break;
	}

	/****** Execute clipping ******/
	ClipperLib::PolyTree result;
	if ( !clpr.Execute( clipType, result ) ) {
		std::cout << "Clipper error: np::Polygon clipping failed.\n";
		return false;
	}

	/****** Create Polygon from PolyTree ******/
	R.contour.resize(result.Total());
	R.is_hole.resize(result.Total());
	R.is_open.resize(result.Total());

	ClipperLib::PolyNode *cnode = result.GetFirst();
	for (int i=0; i<result.Total(); i++) {
		size_t Nv = cnode->Contour.size();
		R.is_hole[i] = cnode->IsHole();
		R.is_open[i] = cnode->IsOpen();
		R.contour[i].resize(Nv);

		for (size_t j=0; j<Nv; j++) {
			#ifndef USE_EXPONENT_SCALING
				R.contour[i][j].x = cnode->Contour[j].X / sc;
				R.contour[i][j].y = cnode->Contour[j].Y / sc;
			#else
				R.contour[i][j].x = std::scalbn(cnode->Contour[j].X, -sc);
				R.contour[i][j].y = std::scalbn(cnode->Contour[j].Y, -sc);
			#endif
		}

		cnode = cnode->GetNext();
	}

	return true;
}
