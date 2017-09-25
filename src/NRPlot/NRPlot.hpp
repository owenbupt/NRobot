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

#ifndef __NRPlot_hpp
#define __NRPlot_hpp

#include <SDL2/SDL.h>
#include <vector>

#include "NRBase.hpp"

/* Colors */
extern SDL_Color BLACK;
extern SDL_Color WHITE;
extern SDL_Color GRAY;
extern SDL_Color GREY;
extern SDL_Color RED;
extern SDL_Color GREEN;
extern SDL_Color BLUE;
extern SDL_Color YELLOW;
extern SDL_Color MAGENTA;
extern SDL_Color ORANGE;
extern SDL_Color CYAN;
extern SDL_Color BROWN;
extern SDL_Color PINK;
/* Color palettes */
extern std::vector<SDL_Color> PLOT_COLORS;

/* Global variables for the plot settings*/
extern int PLOT_WIDTH;
extern int PLOT_HEIGHT;
extern SDL_Color PLOT_BACKGROUND_COLOR;
extern SDL_Color PLOT_AXES_COLOR;
extern SDL_Color PLOT_FOREGROUND_COLOR;
extern double PLOT_SCALE;
extern double PLOT_X_OFFSET;
extern double PLOT_Y_OFFSET;
/* Create global variables for the plot window and renderer */
extern SDL_Window *PLOT_WINDOW;
extern SDL_Renderer *PLOT_RENDERER;


namespace nr {

	int plot_init();
	/* Initialize plot window */

	void plot_quit();
	/* Safely close plot window */

	bool plot_handle_input();
	/* Handle user input, returns true if user wants to quit, else false */

	void plot_render();
	/* Render on screen */

	void plot_clear_render();
	/* Clear the screen */

	void plot_show_axes();
	/* Show the axes */

	void plot_hide_axes();
	/* Hide the axes */

	void plot_point(
		const Point&,
		const SDL_Color& color = PLOT_FOREGROUND_COLOR,
		const int& point_size = 1
	);
	/* Plot a single point. Optionally set its color and size in pixels */

	void plot_points(
		const Points&,
		const SDL_Color& color = PLOT_FOREGROUND_COLOR,
		const int& point_size = 1
	);
	/* Plot a list of points. Optionally set their color and size in pixels */

	void plot_polygon(
		const Polygon&,
		const SDL_Color& color = PLOT_FOREGROUND_COLOR
	);
	/* Plot a single polygon. Optionally set its color */

	void plot_polygon_vertices(
		const Polygon&,
		const SDL_Color& color = PLOT_FOREGROUND_COLOR,
		const int& point_size = 1
	);
	/* Plot the vertices of a single polygon. Optionally set their color and size in pixels */

	void plot_polygons(
		const Polygons&,
		const SDL_Color& color = PLOT_FOREGROUND_COLOR
	);
	/* Plot a list of polygons. Optionally set their color */

	void plot_circle(
		const Circle&,
		const SDL_Color& color = PLOT_FOREGROUND_COLOR
	);
	/* Plot a single circle. Optionally set its color */

	void plot_circles(
		const Circles&,
		const SDL_Color& color = PLOT_FOREGROUND_COLOR
	);
	/* Plot a list of circles. Optionally set their color */

	void plot_ellipse(
		const Ellipse&,
		const SDL_Color& color = PLOT_FOREGROUND_COLOR
	);
	/* Plot a single ellipse. Optionally set its color */

	void plot_segment(
		const Point&,
		const Point&,
		const SDL_Color& color = PLOT_FOREGROUND_COLOR
	);
	/* Plot a line segment defined by two points. Optionally set its color */

	void fill_polygon(
		const Polygon&,
		const SDL_Color& color = PLOT_FOREGROUND_COLOR
	);
	/* Fill a single polygon. Optionally set its color */
}
#endif
