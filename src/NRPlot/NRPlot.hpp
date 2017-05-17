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


#ifndef __NRPlot_hpp
#define __NRPlot_hpp

#include <SDL2/SDL.h>

#include "NRBase.hpp"

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

	int init_SDL();
	/* Initialize SDL */

	void quit_SDL();
	/* Safely quit SDL */

	bool handle_input();
	/* Handle user input, returns true if user wants to quit, else false */


	void plot_render();
	/* Render on screen */

	void clear_render();
	/* Clear the screen */

	void show_axes();
	/* Show the axes */

	void hide_axes();
	/* Hide the axes */

	void plot_point( const Point&, const SDL_Color& color = PLOT_FOREGROUND_COLOR, const int& point_size = 1 );
	/* Plot a single point */

	void plot_points( const Points&, const SDL_Color& color = PLOT_FOREGROUND_COLOR, const int& point_size = 1 );
	/* Plot a list of points */

	void plot_polygon( const Polygon&, const SDL_Color& color = PLOT_FOREGROUND_COLOR );
	/* Plot a single polygon */

	void plot_polygon_vertices( const Polygon&, const SDL_Color& color = PLOT_FOREGROUND_COLOR, const int& point_size = 1 );
	/* Plot the vertices of a single polygon */

	void plot_polygons( const Polygons&, const SDL_Color& color = PLOT_FOREGROUND_COLOR );
	/* Plot a list of polygons */
}
#endif
