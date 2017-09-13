%   Copyright (C) 2017 Sotiris Papatheodorou
%
%   This file is part of NRobot.
%
%   NRobot is free software: you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation, either version 3 of the License, or
%   (at your option) any later version.
%
%   NRobot is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%
%   You should have received a copy of the GNU General Public License
%   along with NRobot.  If not, see <http://www.gnu.org/licenses/>.

clear variables
close all

% Select simulation to load and set plot options
simulation_date = '20170913_231857';
simulation_directory = '..';
PLOT_OBJECTIVE = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Load Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load simulation parameters
sim_params_filename = strjoin(...
{simulation_directory, '/', 'sim_', simulation_date, '_parameters.txt'}, '');
simulation_parameters = load_simulation_parameters( sim_params_filename );
% Number of agents
N = simulation_parameters.N;
% Time vector
t = linspace(0, simulation_parameters.Tfinal, simulation_parameters.iterations);
% Iteration vector
s = 1:500;

% Load agent parameters
agents = cell([1 N]);
for i=1:N
	ID = sprintf('%.4d', i);
	agent_params_filename = strjoin(...
	{'../', 'sim_', simulation_date, '_agent_', ID, '_parameters.txt'}, '');
	agents{i} = load_agent_parameters( agent_params_filename );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if PLOT_OBJECTIVE
	figure
	plot( s, simulation_parameters.H, 'b' );
end








%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load the simulation parameters from the given file and return a struct
% containing them.
function simulation_parameters = load_simulation_parameters( filename )
    % Open file for reading
    f = fopen(filename, 'r');
	if f < 0
		fprintf('Error opening file %s\n', filename);
		simulation_parameters = 1;
		return;
	end

    % Read simulation parameters
    simulation_parameters.N = fscanf(f, '%d', 1);
	simulation_parameters.Tfinal = fscanf(f, '%f', 1);
	simulation_parameters.Tstep = fscanf(f, '%f', 1);
	simulation_parameters.iterations = fscanf(f, '%d', 1);
	simulation_parameters.elapsed_time = fscanf(f, '%f', 1);
	simulation_parameters.average_iteration = fscanf(f, '%f', 1);
	simulation_parameters.H = fscanf(f, '%f', [1 simulation_parameters.iterations]);
	simulation_parameters.H_threshold = fscanf(f, '%f', 1);
	simulation_parameters.H_average_window = fscanf(f, '%d', 1);
	simulation_parameters.region = load_polygon( f );

    % Close file
    fclose(f);
end

% Load the agent parameters from the given file and return a struct
% containing them.
function agent_parameters = load_agent_parameters( filename )
    % Open file for reading
    f = fopen(filename, 'r');
	if f < 0
		fprintf('Error opening file %s\n', filename);
		agent_parameters = 1;
		return;
	end

    % Read simulation parameters
    agent_parameters.ID = fscanf(f, '%d', 1);
	agent_parameters.sensing_radius = fscanf(f, '%f', 1);
	agent_parameters.communication_radius = fscanf(f, '%f', 1);
	agent_parameters.position_uncertainty = fscanf(f, '%f', 1);
	agent_parameters.attitude_uncertainty = fscanf(f, '%f', 1);
	agent_parameters.dynamics = fscanf(f, '%d', 1);
	agent_parameters.time_step = fscanf(f, '%f', 1);
	agent_parameters.partitioning = fscanf(f, '%d', 1);
	agent_parameters.control = fscanf(f, '%d', 1);
	agent_parameters.avoidance = fscanf(f, '%d', 1);
	switch agent_parameters.dynamics
		case 0
			agent_parameters.control_input_gains = fscanf(f, '%f', 2);
		case {1, 2}
			agent_parameters.control_input_gains = fscanf(f, '%f', 3);
		case 3
			agent_parameters.control_input_gains = fscanf(f, '%f', 4);
		otherwise
			fprintf('Invalid dynamics in file %s\n', filename);
			agent_parameters = 1;
			return;
	end
	agent_parameters.base_sensing = load_polygon( f );
	agent_parameters.base_guaranteed_sensing = load_polygon( f );
	agent_parameters.base_relaxed_sensing = load_polygon( f );

    % Close file
    fclose(f);
end

function P = load_polygon( file_ID )
	P = [];
	% Read the number of contours
	Nc = fscanf(file_ID, '%d', 1);
	% Loop over all contours
	for c=1:Nc
		% Read the number of vertices on the contour
		Nv = fscanf(file_ID, '%d', 1);
		% Read the contour hole flag
		is_hole = fscanf(file_ID, '%d', 1);
		% Read the contour open flag
		is_open = fscanf(file_ID, '%d', 1);
		% Read the contour vertices
		C = fscanf(file_ID, '%f', [2 Nv]);
		% MATLAB splits contours by NaNs, makes external contours CW and
		% internal ones CCW
		if is_hole
			[Cx, Cy] = poly2ccw( C(1,:), C(2,:) );
		else
			[Cx, Cy] = poly2cw( C(1,:), C(2,:) );
		end
		
		P = [P [Cx ; Cy]];
		
		if c<Nc
			P = [P [NaN ; NaN]];
		end
	end
end

% plot NaN delimited polygons
function h = plot_poly( P , colorspec )
	if isempty(P)
		return
	end

	a = ishold;

	if nargin == 1
		colorspec = 'b';
	end
	% if the last vertex is not the same as the last, add the first vertex to
	% the end
	% if ~isequal( P(:,1), P(:,end) )
	%     P = [P P(:,1)];
	% end

	x = P(1,:);
	y = P(2,:);

	nan_indices = find( isnan(P(1,:)) ); % The indices are the same for x and y
	indices = [ 0 nan_indices length(x)+1 ];

	h = [];

	for i=1:length( nan_indices )+1

		ht = plot(   [x( indices(i) + 1 : indices(i+1) - 1 ) x(indices(i) + 1)] , ...
				[y( indices(i) + 1 : indices(i+1) - 1 ) y(indices(i) + 1)] , ...
				colorspec );

		h = [h ht];
		hold on;

	end

	% Keep the previous hold state
	if a
		hold on
	else
		hold off
	end

end

% Fill NaN delimited polygons
% 2016/3/13
function H = fill_nan( x , y , colorspec, opac, color )
	if nargin < 4
		opac = 1;
	elseif nargin < 3
		colorspec = 'b';
	end

	nan_indices = find( isnan(x) ); % The indices are the same for x and y
	indices = [ 0 nan_indices length(x)+1 ];

	% Collect the handles of all the plots
	H = zeros(1, length( nan_indices )+1);
	for i=1:length( nan_indices )+1

		if nargin < 5
			h = fill(   x( indices(i) + 1 : indices(i+1) - 1 ) , ...
						y( indices(i) + 1 : indices(i+1) - 1 ) , ...
					colorspec ,'EdgeColor','None','facealpha',opac);
		else
			h = fill(   x( indices(i) + 1 : indices(i+1) - 1 ) , ...
						y( indices(i) + 1 : indices(i+1) - 1 ) , ...
					colorspec, 'FaceColor', color, 'EdgeColor','None','facealpha',opac);
		end

		hold on;
		H(i) = h;
	end
end
