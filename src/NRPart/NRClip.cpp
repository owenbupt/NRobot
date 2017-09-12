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

#include <iostream>
#include <cmath>

#include "clipper.hpp"
#include "NRClip.hpp"


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
		printf("Clipper error: Invalid subject nr::Polygon %p.\n", (void*) &S1);
		return nr::ERROR_INVALID_SUBJECT;
	}
	if ( !clpr.AddPaths(clip, ClipperLib::ptClip, true) ) {
		printf("Clipper error: Invalid clip nr::Polygon %p.\n", (void*) &S2);
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
		std::cout << "Clipper error: nr::Polygon clipping failed.\n";
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
		nr::print(S1);
		return nr::ERROR_INVALID_SUBJECT;
	}
	if ( !clpr.AddPaths(clip, ClipperLib::ptClip, true) ) {
		std::printf("Clipper error: Invalid clip polygon %p.\n", (void*) &S2);
		nr::print(S2);
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
		std::cout << "Clipper error: nr::Polygon clipping failed.\n";
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
