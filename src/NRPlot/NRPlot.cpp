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
#include <SDL2/SDL.h>

#include "NRPlot.hpp"


/********************* Plot initial settings *********************/
int PLOT_WIDTH = 700;
int PLOT_HEIGHT = 700;
SDL_Color PLOT_BACKGROUND_COLOR = {0x44, 0x44, 0x44, 0xFF};
SDL_Color PLOT_AXES_COLOR = {0x30, 0x30, 0x30, 0xFF};
SDL_Color PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
double PLOT_SCALE = 5.0;
double PLOT_X_OFFSET = 0.0;
double PLOT_Y_OFFSET = 0.0;
/*****************************************************************/

#define SCALE_INCREMENT 1.0
#define OFFSET_INCREMENT 0.05


SDL_Window *PLOT_WINDOW = NULL;
SDL_Renderer *PLOT_RENDERER = NULL;



/************************************************************/
/********************* Helper functions *********************/
/************************************************************/

bool handle_keyboard_down(SDL_Event& e) {
	switch (e.key.keysym.sym) {
		/* Q pressed - Quit */
		case SDLK_q:
		return true;

		/* R pressed - Reset view */
		case SDLK_r:
		PLOT_BACKGROUND_COLOR = {0x44, 0x44, 0x44, 0xFF};
		PLOT_AXES_COLOR = {0x30, 0x30, 0x30, 0xFF};
		PLOT_FOREGROUND_COLOR = {0xAA, 0xAA, 0xAA, 0xFF};
		PLOT_SCALE = 5.0;
		PLOT_X_OFFSET = 0.0;
		PLOT_Y_OFFSET = 0.0;
		break;

		/* = pressed - Zoom in */
		case SDLK_EQUALS:
		PLOT_SCALE += SCALE_INCREMENT * 10 * std::sqrt( std::tanh(PLOT_SCALE) );
		break;

		/* - pressed - Zoom out */
		case SDLK_MINUS:
		PLOT_SCALE -= SCALE_INCREMENT * 10 * std::sqrt( std::tanh(PLOT_SCALE) );
		if (PLOT_SCALE < 1) {
			PLOT_SCALE = 1;
		}
		break;

		/* UP pressed - Move up */
		case SDLK_UP:
		PLOT_Y_OFFSET -= OFFSET_INCREMENT;
		break;

		/* DOWN pressed - Move down */
		case SDLK_DOWN:
		PLOT_Y_OFFSET += OFFSET_INCREMENT;
		break;

		/* LEFT pressed - Move left */
		case SDLK_LEFT:
		PLOT_X_OFFSET += OFFSET_INCREMENT;
		break;

		/* RIGHT pressed - Move right */
		case SDLK_RIGHT:
		PLOT_X_OFFSET -= OFFSET_INCREMENT;
		break;
	}

	return false;
}

void handle_window(SDL_Event& e) {
	switch (e.window.event) {
		/* Window resized */
		case SDL_WINDOWEVENT_RESIZED:
		PLOT_WIDTH = e.window.data1;
		PLOT_HEIGHT = e.window.data2;
		break;
	}
}

void handle_mouse_wheel(SDL_Event& e) {
	PLOT_SCALE += e.wheel.y * SCALE_INCREMENT * 10 * std::sqrt( std::tanh(PLOT_SCALE) );
	if (PLOT_SCALE < 1) {
		PLOT_SCALE = 1;
	}
}

void handle_mouse_move(SDL_Event& e) {
	PLOT_X_OFFSET += e.motion.x * OFFSET_INCREMENT;
	PLOT_Y_OFFSET += e.motion.y * OFFSET_INCREMENT;
}


/**********************************************************/
/********************* Main functions *********************/
/**********************************************************/

int nr::init_SDL() {
	/* Initialize SDL */
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
	{
		printf( "SDL could not initialize! SDL_Error: %s\n", SDL_GetError() );
		return(1);
	}
	else
	{

		/* Create window */
		PLOT_WINDOW = SDL_CreateWindow( "NRobot", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, \
		PLOT_WIDTH, PLOT_HEIGHT, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE );

		if( PLOT_WINDOW == NULL )
		{
			printf( "Window could not be created! SDL Error: %s\n", SDL_GetError() );
			return(1);
		}
		else
		{
			/* Create renderer for window */
			PLOT_RENDERER = SDL_CreateRenderer( PLOT_WINDOW, -1, SDL_RENDERER_ACCELERATED );
			if( PLOT_RENDERER == NULL )
			{
				printf( "Renderer could not be created! SDL Error: %s\n", SDL_GetError() );
				return(1);
			}
			else
			{
				/* Initialize renderer color and fill the window */
				SDL_SetRenderDrawColor( PLOT_RENDERER,
					PLOT_BACKGROUND_COLOR.r,
					PLOT_BACKGROUND_COLOR.g,
					PLOT_BACKGROUND_COLOR.b,
					PLOT_BACKGROUND_COLOR.a );
				SDL_RenderClear( PLOT_RENDERER );
			}
		}
	}
	/* Successful initialization */
	return(0);
}

void nr::quit_SDL() {
	/* Destroy the window and renderer */
	SDL_DestroyRenderer( PLOT_RENDERER );
	SDL_DestroyWindow( PLOT_WINDOW );
	PLOT_RENDERER = NULL;
	PLOT_WINDOW = NULL;

	/* Quit SDL */
	SDL_Quit();
}

bool nr::handle_input() {
	SDL_Event e;
	/* Wait for quit event */
	while ( SDL_PollEvent( &e ) ) {
		/* X button pressed */
		if ( e.type == SDL_QUIT ) {
			return true;
		/* Key pressed */
		} else if( e.type == SDL_KEYDOWN ) {
			return handle_keyboard_down(e);
		/* Window event */
		} else if (e.type == SDL_WINDOWEVENT) {
			handle_window(e);
		/* Mouse wheel */
		} else if (e.type == SDL_MOUSEWHEEL) {
			handle_mouse_wheel(e);
		/* Mouse movement */
		} else if (e.type == SDL_MOUSEMOTION) {
			// handle_mouse_move(e);
		}
	}
	return false;
}


void nr::plot_render() {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		PLOT_BACKGROUND_COLOR.r,
		PLOT_BACKGROUND_COLOR.g,
		PLOT_BACKGROUND_COLOR.b,
		PLOT_BACKGROUND_COLOR.a );
	SDL_RenderPresent( PLOT_RENDERER );
}

void nr::clear_render() {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		PLOT_BACKGROUND_COLOR.r,
		PLOT_BACKGROUND_COLOR.g,
		PLOT_BACKGROUND_COLOR.b,
		PLOT_BACKGROUND_COLOR.a );
	SDL_RenderClear( PLOT_RENDERER );
}

void nr::show_axes() {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		PLOT_AXES_COLOR.r,
		PLOT_AXES_COLOR.g,
		PLOT_AXES_COLOR.b,
		PLOT_AXES_COLOR.a );
		SDL_RenderDrawLine( PLOT_RENDERER,
			PLOT_WIDTH/2.0 + PLOT_SCALE * PLOT_X_OFFSET,
			0,
			PLOT_WIDTH/2.0 + PLOT_SCALE * PLOT_X_OFFSET,
			PLOT_HEIGHT );
		SDL_RenderDrawLine( PLOT_RENDERER,
			0,
			PLOT_HEIGHT/2.0 - PLOT_SCALE * PLOT_Y_OFFSET,
			PLOT_WIDTH,
			PLOT_HEIGHT/2.0 - PLOT_SCALE * PLOT_Y_OFFSET );
}

void nr::hide_axes() {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		PLOT_BACKGROUND_COLOR.r,
		PLOT_BACKGROUND_COLOR.g,
		PLOT_BACKGROUND_COLOR.b,
		PLOT_BACKGROUND_COLOR.a );
		SDL_RenderDrawLine( PLOT_RENDERER,
			PLOT_WIDTH/2.0 + PLOT_SCALE * PLOT_X_OFFSET,
			0,
			PLOT_WIDTH/2.0 + PLOT_SCALE * PLOT_X_OFFSET,
			PLOT_HEIGHT );
		SDL_RenderDrawLine( PLOT_RENDERER,
			0,
			PLOT_HEIGHT/2.0 - PLOT_SCALE * PLOT_Y_OFFSET,
			PLOT_WIDTH,
			PLOT_HEIGHT/2.0 - PLOT_SCALE * PLOT_Y_OFFSET );
}

void nr::plot_point( const nr::Point& A, const SDL_Color& color, const int& point_size ) {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		color.r,
		color.g,
		color.b,
		color.a );
	for (int k=-point_size; k<=point_size; k++) {
		for (int l=-point_size; l<=point_size; l++) {
			SDL_RenderDrawPoint( PLOT_RENDERER,
				PLOT_WIDTH/2.0 + PLOT_SCALE * (A.x + PLOT_X_OFFSET) + k,
				PLOT_HEIGHT/2.0 - PLOT_SCALE * (A.y + PLOT_Y_OFFSET) + l );
		}
	}
	// SDL_RenderDrawPoint( PLOT_RENDERER,
	// 	PLOT_WIDTH/2.0 + PLOT_SCALE * (A.x + PLOT_X_OFFSET),
	// 	PLOT_HEIGHT/2.0 - PLOT_SCALE * (A.y + PLOT_Y_OFFSET) );
}

void nr::plot_points( const nr::Points& A, const SDL_Color& color, const int& point_size ) {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		color.r,
		color.g,
		color.b,
		color.a );

	/* Loop over all points */
	for (size_t i=0; i<A.size(); i++) {
		for (int k=-point_size; k<=point_size; k++) {
			for (int l=-point_size; l<=point_size; l++) {
				SDL_RenderDrawPoint( PLOT_RENDERER,
					PLOT_WIDTH/2.0 + PLOT_SCALE * (A[i].x + PLOT_X_OFFSET) + k,
					PLOT_HEIGHT/2.0 - PLOT_SCALE * (A[i].y + PLOT_Y_OFFSET) + l );
			}
		}
		// SDL_RenderDrawPoint( PLOT_RENDERER,
		// 	PLOT_WIDTH/2.0 + PLOT_SCALE * (A[i].x + PLOT_X_OFFSET),
		// 	PLOT_HEIGHT/2.0 - PLOT_SCALE * (A[i].y + PLOT_Y_OFFSET) );
	}
}

void nr::plot_polygon( const nr::Polygon& P, const SDL_Color& color ) {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		color.r,
		color.g,
		color.b,
		color.a );

	/* Loop over all contours */
	for (size_t i=0; i<P.contour.size(); i++) {
		/* Loop over all vertex pairs */
		size_t Nv = P.contour[i].size();
		for (size_t j=0; j<Nv; j++) {
			size_t jj = (j+1) % Nv;

			/* Draw the line between the vertices */
			/* Note that the y axis is inverted */
			SDL_RenderDrawLine( PLOT_RENDERER,
				PLOT_WIDTH/2.0 + PLOT_SCALE * (P.contour[i].at(j).x + PLOT_X_OFFSET),
				PLOT_HEIGHT/2.0 - PLOT_SCALE * (P.contour[i].at(j).y + PLOT_Y_OFFSET),
				PLOT_WIDTH/2.0 + PLOT_SCALE * (P.contour[i].at(jj).x + PLOT_X_OFFSET),
				PLOT_HEIGHT/2.0 - PLOT_SCALE * (P.contour[i].at(jj).y + PLOT_Y_OFFSET) );
		}
	}
}

void nr::plot_polygon_vertices( const nr::Polygon& P, const SDL_Color& color, const int& point_size ) {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		color.r,
		color.g,
		color.b,
		color.a );

	/* Loop over all contours */
	for (size_t i=0; i<P.contour.size(); i++) {
		/* Loop over all vertices */
		for (size_t j=0; j<P.contour[i].size(); j++) {
			for (int k=-point_size; k<=point_size; k++) {
				for (int l=-point_size; l<=point_size; l++) {
					SDL_RenderDrawPoint( PLOT_RENDERER,
						PLOT_WIDTH/2.0 + PLOT_SCALE * (P.contour[i][j].x + PLOT_X_OFFSET) + k,
						PLOT_HEIGHT/2.0 - PLOT_SCALE * (P.contour[i][j].y + PLOT_Y_OFFSET) + l );
				}
			}
			// SDL_RenderDrawPoint( PLOT_RENDERER,
			// 	PLOT_WIDTH/2.0 + PLOT_SCALE * (P.contour[i][j].x + PLOT_X_OFFSET),
			// 	PLOT_HEIGHT/2.0 - PLOT_SCALE * (P.contour[i][j].y + PLOT_Y_OFFSET) );
		}
	}
}

void nr::plot_polygons( const nr::Polygons& P, const SDL_Color& color ) {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		color.r,
		color.g,
		color.b,
		color.a );

	/* Loop over all polygons */
	for (size_t k=0; k<P.size(); k++) {
		/* Loop over all contours */
		for (size_t i=0; i<P[k].contour.size(); i++) {
			/* Loop over all vertex pairs */
			size_t Nv = P[k].contour[i].size();
			for (size_t j=0; j<Nv; j++) {
				size_t jj = (j+1) % Nv;

				/* Draw the line between the vertices */
				/* Note that the y axis is inverted */
				SDL_RenderDrawLine( PLOT_RENDERER,
					PLOT_WIDTH/2.0 + PLOT_SCALE * (P[k].contour[i].at(j).x + PLOT_X_OFFSET),
					PLOT_HEIGHT/2.0 - PLOT_SCALE * (P[k].contour[i].at(j).y + PLOT_Y_OFFSET),
					PLOT_WIDTH/2.0 + PLOT_SCALE * (P[k].contour[i].at(jj).x + PLOT_X_OFFSET),
					PLOT_HEIGHT/2.0 - PLOT_SCALE * (P[k].contour[i].at(jj).y + PLOT_Y_OFFSET) );
			}
		}
	}
}
