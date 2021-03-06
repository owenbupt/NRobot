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

#include "NRPlot.hpp"


/********************* Plot initial settings *********************/
int PLOT_WIDTH = 700;
int PLOT_HEIGHT = 700;
double PLOT_SCALE = 5.0;
double PLOT_X_OFFSET = 0.0;
double PLOT_Y_OFFSET = 0.0;
/*****************************************************************/

/* Colors */
// SDL_Color BLACK = {0x00, 0x00, 0x00, 0xFF};
// SDL_Color WHITE = {0xFF, 0xFF, 0xFF, 0xFF};
// SDL_Color GRAY = {0x77, 0x77, 0x77, 0xFF};
// SDL_Color GREY = GRAY;
// SDL_Color RED = {0xFF, 0x00, 0x00, 0xFF};
// SDL_Color GREEN = {0x00, 0xFF, 0x00, 0xFF};
// SDL_Color BLUE = {0x00, 0x00, 0xFF, 0xFF};
// SDL_Color YELLOW = {0xFF, 0xFF, 0x00, 0xFF};
// SDL_Color MAGENTA = {0xFF, 0x00, 0xFF, 0xFF};
// SDL_Color CYAN = {0x00, 0xFF, 0xFF, 0xFF};
// SDL_Color ORANGE = {0xFF, 0x77, 0x00, 0xFF};
// SDL_Color BROWN = {0xAB, 0x59, 0x04, 0xFF};
// SDL_Color PINK = {0xFF, 0x00, 0x77, 0xFF};
// SDL_Color PLOT_BACKGROUND_COLOR = WHITE;
// SDL_Color PLOT_AXES_COLOR = GRAY;
// SDL_Color PLOT_FOREGROUND_COLOR = BLACK;
//
// std::vector<SDL_Color> PLOT_COLORS = { RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, ORANGE, BROWN, PINK };

/* https://www.romanzolotarev.com/pico-8-color-palette/ */
SDL_Color BLACK = {0x00, 0x00, 0x00, 0xFF};
SDL_Color WHITE = {0xFF, 0xFF, 0xFF, 0xFF};
SDL_Color GRAY = {0xC2, 0xC3, 0xC7, 0xFF};
SDL_Color GREY = GRAY;
SDL_Color DARK_GRAY = {0x5F, 0x57, 0x4F, 0xFF};
SDL_Color DARK_GREY = DARK_GRAY;
SDL_Color RED = {0xFF, 0x00, 0x40, 0xFF};
SDL_Color GREEN = {0x00, 0xE4, 0x3B, 0xFF};
SDL_Color BLUE = {0x29, 0xAD, 0xFF, 0xFF};
SDL_Color YELLOW = {0xFF, 0xEC, 0x27, 0xFF};
SDL_Color PURPLE = {0x7E, 0x25, 0x53, 0xFF};
SDL_Color ORANGE = {0xFF, 0xA3, 0x00, 0xFF};
SDL_Color BROWN = {0xAB, 0x52, 0x3B, 0xFF};
SDL_Color PINK = {0xFF, 0x77, 0xAB, 0xFF};
SDL_Color DARK_GREEN = {0x00, 0x87, 0x51, 0xFF};
SDL_Color DARK_BLUE = {0x1D, 0x2E, 0x53, 0xFF};
SDL_Color CREAM = {0xFF, 0xF1, 0xE8, 0xFF};
SDL_Color PLOT_BACKGROUND_COLOR = WHITE;
SDL_Color PLOT_AXES_COLOR = GRAY;
SDL_Color PLOT_FOREGROUND_COLOR = BLACK;

std::vector<SDL_Color> PLOT_COLORS = { RED, GREEN, BLUE, YELLOW, PURPLE, ORANGE, BROWN, PINK, DARK_GREEN, DARK_BLUE };

#define SCALE_INCREMENT 0.3
#define OFFSET_INCREMENT 5


SDL_Window *PLOT_WINDOW = NULL;
SDL_Renderer *PLOT_RENDERER = NULL;



/************************************************************/
/********************* Helper functions *********************/
/************************************************************/

bool nr_handle_keyboard_down(SDL_Event& e) {
	switch (e.key.keysym.sym) {
		/* Q pressed - Quit */
		case SDLK_q:
		return true;

		/* R pressed - Reset view */
		case SDLK_r:
		PLOT_BACKGROUND_COLOR = WHITE;
		PLOT_AXES_COLOR = GRAY;
		PLOT_FOREGROUND_COLOR = BLACK;
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
		PLOT_Y_OFFSET += OFFSET_INCREMENT;
		break;

		/* DOWN pressed - Move down */
		case SDLK_DOWN:
		PLOT_Y_OFFSET -= OFFSET_INCREMENT;
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

void nr_handle_window(SDL_Event& e) {
	switch (e.window.event) {
		/* Window resized */
		case SDL_WINDOWEVENT_RESIZED:
		PLOT_WIDTH = e.window.data1;
		PLOT_HEIGHT = e.window.data2;
		break;
	}
}

void nr_handle_mouse_wheel(SDL_Event& e) {
	PLOT_SCALE += e.wheel.y * SCALE_INCREMENT * 10 * std::sqrt( std::tanh(PLOT_SCALE) );
	if (PLOT_SCALE < 1) {
		PLOT_SCALE = 1;
	}
}

void nr_handle_mouse_move(SDL_Event& e) {
	PLOT_X_OFFSET -= e.motion.xrel * OFFSET_INCREMENT;
	PLOT_Y_OFFSET -= e.motion.yrel * OFFSET_INCREMENT;
}

void nr_handle_mouse_button(SDL_Event& e) {
	/* Get values relative to the window center */
	PLOT_X_OFFSET -= e.button.x - PLOT_WIDTH/2.0;
	PLOT_Y_OFFSET -= e.button.y - PLOT_HEIGHT/2.0;
}


/**********************************************************/
/********************* Main functions *********************/
/**********************************************************/

int nr::plot_init() {
	/* Initialize SDL */
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 ) {
		printf( "SDL could not initialize! SDL_Error: %s\n", SDL_GetError() );
		return(1);
	} else {

		/* Create window */
		PLOT_WINDOW = SDL_CreateWindow( "NRobot", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, \
		PLOT_WIDTH, PLOT_HEIGHT, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE );

		if( PLOT_WINDOW == NULL ) {
			printf( "Window could not be created! SDL Error: %s\n", SDL_GetError() );
			return(1);
		} else {
			/* Create renderer for window */
			PLOT_RENDERER = SDL_CreateRenderer( PLOT_WINDOW, -1, SDL_RENDERER_ACCELERATED );
			if( PLOT_RENDERER == NULL ) {
				printf( "Renderer could not be created! SDL Error: %s\n", SDL_GetError() );
				return(1);
			} else {
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

void nr::plot_quit() {
	/* Destroy the window and renderer */
	SDL_DestroyRenderer( PLOT_RENDERER );
	SDL_DestroyWindow( PLOT_WINDOW );
	PLOT_RENDERER = NULL;
	PLOT_WINDOW = NULL;

	/* Quit SDL */
	SDL_Quit();
}

bool nr::plot_handle_input() {
	SDL_Event e;
	/* Wait for quit event */
	while ( SDL_PollEvent( &e ) ) {
		/* X button pressed */
		if ( e.type == SDL_QUIT ) {
			return true;
		/* Key pressed */
		} else if( e.type == SDL_KEYDOWN ) {
			return nr_handle_keyboard_down(e);
		/* Window event */
		} else if (e.type == SDL_WINDOWEVENT) {
			nr_handle_window(e);
		/* Mouse wheel */
		} else if (e.type == SDL_MOUSEWHEEL) {
			nr_handle_mouse_wheel(e);
		/* Mouse movement */
		} else if (e.type == SDL_MOUSEMOTION) {
			// nr_handle_mouse_move(e);
		/* Mouse click */
		} else if (e.type == SDL_MOUSEBUTTONDOWN) {
			nr_handle_mouse_button(e);
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

void nr::plot_clear_render() {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		PLOT_BACKGROUND_COLOR.r,
		PLOT_BACKGROUND_COLOR.g,
		PLOT_BACKGROUND_COLOR.b,
		PLOT_BACKGROUND_COLOR.a );
	SDL_RenderClear( PLOT_RENDERER );
}

void nr::plot_show_axes() {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		PLOT_AXES_COLOR.r,
		PLOT_AXES_COLOR.g,
		PLOT_AXES_COLOR.b,
		PLOT_AXES_COLOR.a );
		SDL_RenderDrawLine( PLOT_RENDERER,
			PLOT_WIDTH/2.0 + PLOT_X_OFFSET,
			0,
			PLOT_WIDTH/2.0 + PLOT_X_OFFSET,
			PLOT_HEIGHT );
		SDL_RenderDrawLine( PLOT_RENDERER,
			0,
			PLOT_HEIGHT/2.0 + PLOT_Y_OFFSET,
			PLOT_WIDTH,
			PLOT_HEIGHT/2.0 + PLOT_Y_OFFSET );
}

void nr::plot_hide_axes() {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		PLOT_BACKGROUND_COLOR.r,
		PLOT_BACKGROUND_COLOR.g,
		PLOT_BACKGROUND_COLOR.b,
		PLOT_BACKGROUND_COLOR.a );
		SDL_RenderDrawLine( PLOT_RENDERER,
			PLOT_WIDTH/2.0 + PLOT_X_OFFSET,
			0,
			PLOT_WIDTH/2.0 + PLOT_X_OFFSET,
			PLOT_HEIGHT );
		SDL_RenderDrawLine( PLOT_RENDERER,
			0,
			PLOT_HEIGHT/2.0 + PLOT_Y_OFFSET,
			PLOT_WIDTH,
			PLOT_HEIGHT/2.0 + PLOT_Y_OFFSET );
}

void nr::plot_point(
	const nr::Point& A,
	const SDL_Color& color,
	const int& point_size
) {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		color.r,
		color.g,
		color.b,
		color.a );
	for (int k=-point_size; k<=point_size; k++) {
		for (int l=-point_size; l<=point_size; l++) {
			SDL_RenderDrawPoint( PLOT_RENDERER,
				PLOT_WIDTH/2.0 + PLOT_SCALE * A.x + PLOT_X_OFFSET + k,
				PLOT_HEIGHT/2.0 - PLOT_SCALE * A.y + PLOT_Y_OFFSET + l );
		}
	}
}

void nr::plot_points(
	const nr::Points& A,
	const SDL_Color& color,
	const int& point_size
) {
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
					PLOT_WIDTH/2.0 + PLOT_SCALE * A[i].x + PLOT_X_OFFSET + k,
					PLOT_HEIGHT/2.0 - PLOT_SCALE * A[i].y + PLOT_Y_OFFSET + l );
			}
		}
	}
}

void nr::plot_polygon(
	const nr::Polygon& P,
	const SDL_Color& color
) {
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
				PLOT_WIDTH/2.0 + PLOT_SCALE * P.contour[i].at(j).x + PLOT_X_OFFSET,
				PLOT_HEIGHT/2.0 - PLOT_SCALE * P.contour[i].at(j).y + PLOT_Y_OFFSET,
				PLOT_WIDTH/2.0 + PLOT_SCALE * P.contour[i].at(jj).x + PLOT_X_OFFSET,
				PLOT_HEIGHT/2.0 - PLOT_SCALE * P.contour[i].at(jj).y + PLOT_Y_OFFSET );
		}
	}
}

void nr::plot_polygon_vertices(
	const nr::Polygon& P,
	const SDL_Color& color,
	const int& point_size
) {
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
						PLOT_WIDTH/2.0 + PLOT_SCALE * P.contour[i][j].x + PLOT_X_OFFSET + k,
						PLOT_HEIGHT/2.0 - PLOT_SCALE * P.contour[i][j].y + PLOT_Y_OFFSET + l );
				}
			}
		}
	}
}

void nr::plot_polygons(
	const nr::Polygons& P,
	const SDL_Color& color
) {
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
					PLOT_WIDTH/2.0 + PLOT_SCALE * P[k].contour[i].at(j).x + PLOT_X_OFFSET,
					PLOT_HEIGHT/2.0 - PLOT_SCALE * P[k].contour[i].at(j).y + PLOT_Y_OFFSET,
					PLOT_WIDTH/2.0 + PLOT_SCALE * P[k].contour[i].at(jj).x + PLOT_X_OFFSET,
					PLOT_HEIGHT/2.0 - PLOT_SCALE * P[k].contour[i].at(jj).y + PLOT_Y_OFFSET );
			}
		}
	}
}

void nr::plot_circle(
	const nr::Circle& C,
	const SDL_Color& color
) {
	/* Create a polygon from the circle */
	nr::Polygon circle_poly;
	circle_poly = nr::Polygon( C );

	/* Plot the polygon */
	nr::plot_polygon( circle_poly, color );
}

void nr::plot_circles(
	const nr::Circles& C,
	const SDL_Color& color
) {
	/* Create a polygon list from the circles */
	nr::Polygons circle_polys;
	circle_polys = nr::Polygons( C );

	/* Plot the polygon */
	nr::plot_polygons( circle_polys, color );
}

void nr::plot_ellipse(
	const nr::Ellipse& E,
	const SDL_Color& color
) {
	/* Create a polygon from the ellipse */
	nr::Polygon ellipse_poly;
	ellipse_poly = nr::Polygon( E );

	/* Plot the polygon */
	nr::plot_polygon( ellipse_poly, color );
}

void nr::plot_segment(
	const nr::Point& P1,
	const nr::Point& P2,
	const SDL_Color& color
) {
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		color.r,
		color.g,
		color.b,
		color.a );

	/* Draw the line between the vertices */
	/* Note that the y axis is inverted */
	SDL_RenderDrawLine( PLOT_RENDERER,
		PLOT_WIDTH/2.0 + PLOT_SCALE * P1.x + PLOT_X_OFFSET,
		PLOT_HEIGHT/2.0 - PLOT_SCALE * P1.y + PLOT_Y_OFFSET,
		PLOT_WIDTH/2.0 + PLOT_SCALE * P2.x + PLOT_X_OFFSET,
		PLOT_HEIGHT/2.0 - PLOT_SCALE * P2.y + PLOT_Y_OFFSET );
}

void nr::fill_polygon(
	const nr::Polygon& P,
	const SDL_Color& color
) {
	std::printf("fill_polygon() IS NOT YET WORKING.\n");
	/* Store the background color. Used for filling holes. */
	SDL_Color bg_color;
	SDL_GetRenderDrawColor( PLOT_RENDERER,
		&(bg_color.r),
		&(bg_color.g),
		&(bg_color.b),
		&(bg_color.a) );
	/* Set the renderer color to the foreground color. */
	SDL_SetRenderDrawColor( PLOT_RENDERER,
		color.r,
		color.g,
		color.b,
		color.a );

	/* Minimum and maximum polygon y values in pixels. */
	double miny_px = PLOT_HEIGHT;
	double maxy_px = 0;
	/* Convert coordinates to pixels. */
	nr::Polygon P_px = P;
	/* Loop over all contours. */
	for (size_t i=0; i<P.contour.size(); i++) {
		/* Loop over all vertices. */
		size_t Nv = P.contour[i].size();
		for (size_t j=0; j<Nv; j++) {
			/* Scale and translate vertices. */
			P_px.contour[i][j].x = PLOT_WIDTH/2.0 +
			PLOT_SCALE * P.contour[i][j].x + PLOT_X_OFFSET;
			P_px.contour[i][j].y = PLOT_HEIGHT/2.0 -
			PLOT_SCALE * P.contour[i][j].y + PLOT_Y_OFFSET;
			/* Find min and max vertex y coordinates in pixels. */
			if (P_px.contour[i][j].y < miny_px) {
				miny_px = P_px.contour[i][j].y;
			}
			if (P_px.contour[i][j].y > maxy_px) {
				maxy_px = P_px.contour[i][j].y;
			}
		}
	}

	/* Loop over all polygon pixel rows. */
	for (size_t r=(size_t) miny_px; r<=(size_t) maxy_px; r++) {
		/* Create a vector of intersection x coordinates in pixels. */
		/* Find intersections. */
		/* Sort intersections. */
		/* Fill between intersections. */
	}

	/* Plot polygon edges */
	for (size_t i=0; i<P_px.contour.size(); i++) {
		/* Loop over all vertex pairs */
		size_t Nv = P_px.contour[i].size();
		for (size_t j=0; j<Nv; j++) {
			size_t jj = (j+1) % Nv;

			/* Draw the line between the vertices */
			/* Note that the y axis is inverted */
			SDL_RenderDrawLine( PLOT_RENDERER,
				P_px.contour[i].at(j).x,
				P_px.contour[i].at(j).y,
				P_px.contour[i].at(jj).x,
				P_px.contour[i].at(jj).y);
		}
	}
}
