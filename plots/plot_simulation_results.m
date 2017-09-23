% Copyright 2017 Sotiris Papatheodorou
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%    http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

clear variables
close all

colors = [ 255, 0, 77 ;
           0, 228, 54 ;
		   41, 173, 255 ;
		   255, 163, 0 ;
		   126, 37, 83 ;
		   0, 135, 81 ;
		   171, 82, 54 ;
		   255, 119, 168 ] ./ 255;

% Select simulation to load and set plot options
simulation_date = '20170924_022009'; % Guaranteed only
simulation_directory = '..';
% simulation_date = '20170915_044029'; % Relaxed -> guaranteed
% simulation_directory = '/home/sotiris/Dropbox/Conferences/2018/ICRA18_PT/Figures/Sim2/Results';

PLOT_OBJECTIVE = 1;
PLOT_GUARANTEED_OBJECTIVE = 0;
PLOT_TRAJECTORIES = 1;
PLOT_INITIAL_STATE = 0;
PLOT_FINAL_STATE = 0;
CALCULATE_REAL_FINAL_COVERAGE = 0;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Load Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load simulation parameters
sim_params_filename = strjoin(...
{simulation_directory, '/', 'sim_', simulation_date, '_parameters.txt'}, '');
simulation_parameters = load_simulation_parameters( sim_params_filename );
% Number of agents
N = simulation_parameters.N;
% Number of iterations
iterations = simulation_parameters.iterations;
% Time vector
t = linspace(0, simulation_parameters.Tfinal, iterations);
% Iteration vector
s = 1:iterations;
% Axis scale for region
simulation_parameters.axis = [0 3 -0.5 2.5];

% Load agent parameters
agents = cell([1 N]);
for i=1:N
	ID = sprintf('%.4d', i);
	agent_params_filename = strjoin(...
	{simulation_directory, '/', 'sim_', simulation_date, '_agent_', ID, '_parameters.txt'}, '');
	agents{i} = load_agent_parameters( agent_params_filename );
end

% Load agent state
agent_state = cell([1 N]);
for i=1:N
	ID = sprintf('%.4d', i);
	agent_state_filename = strjoin(...
	{simulation_directory, '/', 'sim_', simulation_date, '_agent_', ID, '_state.txt'}, '');
	agent_state{i} = load_agent_state( agent_state_filename );
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Text parameters
text_size = 15;

if PLOT_OBJECTIVE
	figure('Name','Objective');
	plot( t, simulation_parameters.H, 'b' );
end

if PLOT_GUARANTEED_OBJECTIVE
	% Vector with iterations at which agents converge
	switched_law = zeros(1,N);
	Hg = zeros(1,iterations);
	% Compute the guaranteed objective for each iteration
	for s=1:iterations
		% Compute union of sensing regions
		ux = [];
		uy = [];
		for i=1:N
			% Create sensing
			sensing = rot( agents{i}.base_guaranteed_sensing, ...
			agent_state{i}.yaw(s) ) + ...
			[agent_state{i}.x(s) ; agent_state{i}.y(s)];
			% Add to union
			[ux, uy] = polybool('or', ux, uy, sensing(1,:), sensing(2,:));
			% Check convergence
			if (s > 1) && ...
			(agent_state{i}.relaxed_sensing_quality(s-1) ~= ...
			agent_state{i}.relaxed_sensing_quality(s))
				switched_law(i) = s;
			end
		end
		% Intersect with region
		[ux, uy] = polybool('and', ux, uy, ...
		simulation_parameters.region(1,:), ...
		simulation_parameters.region(2,:));
		% Add union area to H
		Hg(s) = polyarea_nan(ux, uy);
		fprintf('Iteration %d\n',s);
	end
	% Calculate maximum guaranteed objective value
	Hg_max = 0;
	for i=1:N
		Hg_max = Hg_max + polyarea( agents{i}.base_guaranteed_sensing(1,:), ...
		agents{i}.base_guaranteed_sensing(2,:) );
	end
	
	figure('Name','Guaranteed Objective');
	hold on
	plot( t, 100*Hg/Hg_max, 'b' );
	plot( t, 100*ones(size(t)), 'k--' );
	% Plot vertical lines at convergence
	for i=1:N
		plot( [t(switched_law(i)) t(switched_law(i))], ...
			[0 100*Hg(switched_law(i))/Hg_max], 'k');
	end
	axis([t(1) t(end) 0 100]);
	xlabel('Time');
	ylabel('H (% of maximum)');
end

if PLOT_TRAJECTORIES
	figure('Name','Trajectories');
	hold on
	axis equal
	axis off
	axis(simulation_parameters.axis);
	plot_poly( simulation_parameters.region, 'k' );
	for i=1:N
		plot( agent_state{i}.x, agent_state{i}.y, 'b', 'Color', colors(i,:) );
		plot( agent_state{i}.x(1), agent_state{i}.y(1), 'b.', 'Color', colors(i,:) );
		plot( agent_state{i}.x(end), agent_state{i}.y(end), 'bo', 'Color', colors(i,:) );
	end
end

if PLOT_INITIAL_STATE
	figure('Name','Initial State');
	plot_state( simulation_parameters, agents, agent_state, 1, colors );
end

if PLOT_FINAL_STATE
	figure('Name','Final State');
	plot_state( simulation_parameters, agents, agent_state, iterations, colors );
end

if CALCULATE_REAL_FINAL_COVERAGE
	s = iterations;
	% Calculate maximum possible real coverage
	H_max = 0;
	for i=1:N
		H_max = H_max + polyarea( agents{i}.base_sensing(1,:), ...
		agents{i}.base_sensing(2,:) );
	end
	H_low = H_max;
	H_high = 0;
	% Calculate real coverage at M random final states
	M = 100;
	for k=1:M
		% Compute union of sensing regions
		ux = [];
		uy = [];
		for i=1:N
			% Generate a random position and orientation within the uncertainty bounds
			magnitude = agents{i}.position_uncertainty * rand();
			angle = 2*pi * rand();
			[x, y] = pol2cart( angle, magnitude );
			agents{i}.random_position = [x ; y];
			agents{i}.random_attitude = ...
			agents{i}.attitude_uncertainty * 2*(rand()-0.5);
			% Create sensing
			sensing = rot( agents{i}.base_sensing, ...
			agent_state{i}.yaw(s) + agents{i}.random_attitude ) + ...
			[agent_state{i}.x(s) ; agent_state{i}.y(s)] + agents{i}.random_position;
			% Add to union
			[ux, uy] = polybool('or', ux, uy, sensing(1,:), sensing(2,:));
		end
		% Intersect with region
		[ux, uy] = polybool('and', ux, uy, ...
		simulation_parameters.region(1,:), ...
		simulation_parameters.region(2,:));
		% Add union area to H
		H = polyarea_nan(ux, uy);
		if H < H_low
			H_low = H;
		end
		if H > H_high
			H_high = H;
		end
	end
	
	fprintf('Real coverage at final state %.2f %% - %.2f %% as computed by %d random configurations\n', ...
	100*H_low/H_max, 100*H_high/H_max, M);
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

    % Read agent parameters
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
		case {0, 4}
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
	% Create total sensing region by getting the first contour of the relaxed
	% sensing region
	first_contour_end = ...
	find(isnan(agent_parameters.base_relaxed_sensing(1,:)));
	agent_parameters.base_total_sensing = ...
	agent_parameters.base_relaxed_sensing(:,1:first_contour_end-1);
	% Generate a random position and orientation within the uncertainty bounds
	magnitude = agent_parameters.position_uncertainty * rand();
	angle = 2*pi * rand();
	[x, y] = pol2cart( angle, magnitude );
	agent_parameters.random_position = [x ; y];
	agent_parameters.random_attitude = ...
	agent_parameters.attitude_uncertainty * 2*(rand()-0.5);

    % Close file
    fclose(f);
end

% Load the agent state from the given file and return a struct
% containing it.
function agent_state = load_agent_state( filename )
    % Read file contents as a matrix
	d = importdata( filename );
	
	% Put data into matrices
	agent_state.x = d(:,2);
	agent_state.y = d(:,3);
	agent_state.z = d(:,4);
	agent_state.roll = d(:,5);
	agent_state.pitch = d(:,6);
	agent_state.yaw = d(:,7);
	agent_state.x_velocity = d(:,8);
	agent_state.y_velocity = d(:,9);
	agent_state.z_velocity = d(:,10);
	agent_state.roll_velocity = d(:,11);
	agent_state.pitch_velocity = d(:,12);
	agent_state.yaw_velocity = d(:,13);
	agent_state.relaxed_sensing_quality = d(:,14);
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

% Rotates the coordinates of A by theta (in radians)
function B = rot(A,theta)

R = [    cos(theta)  -sin(theta);
            sin(theta)  cos(theta)];
B = zeros( size(A) );
N = length(A(1,:));
for i=1:N
    B(:,i) = R * A(:,i);
end
end

% Returns the cells for the given sensing patterns constrained inside the region
function W = partition( region, sensing )
	N = length(sensing);
	W = cell([1 N]);
	
	% Loop over each sensing
	for i=1:N
		% Initialize the cell to the sensing
		W{i} = sensing{i};
		% Subtract all other sensing
		for j=1:N
			if i ~= j
				if ~isempty( W{i} )
					[pbx, pby] = ...
					polybool('minus', W{i}(1,:), W{i}(2,:), ...
					sensing{j}(1,:), sensing{j}(2,:));
					W{i} = [pbx ; pby];
				else
					break;
				end
			end
		end
		% Intersect with the region
		if ~isempty( W{i} )
			[pbx, pby] = ...
			polybool('and', region(1,:), region(2,:), W{i}(1,:), W{i}(2,:));
			W{i} = [pbx ; pby];
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

function h = plot_uncertainty( p, P , o, O, color )
	% Save hold state
	a = ishold;
	hold on
	% Default color
	if nargin == 2
		color = 'b';
	end

	% Positioning uncertainty
	h1 = plot_poly( p, strcat(color,'.') );
	h2 = plot_circle( p , P , color);

	% Orientation uncertainty
	heading = zeros(2,1);
	heading1 = zeros(2,1);
	heading2 = zeros(2,1);
	[heading(1), heading(2)] = pol2cart( o, 2*P );
	[heading1(1), heading1(2)] = pol2cart( o-O, P );
	[heading2(1), heading2(2)] = pol2cart( o+O, P );
	h3 = plot_poly( [p heading+p], color );
	h4 = plot_poly( [p heading1+p], strcat(color,'-') );
	h5 = plot_poly( [p heading2+p], strcat(color,'-') );

	h = [ h1 h2 h3 h4 h5 ];
	% Restore hold state
	if a
		hold on
	else
		hold off
	end
end

% Plots a circle with center O and radius r
function plot_handle = plot_circle( O , r , linespec)

if nargin < 3
    linespec = 'b';
end

t = linspace(0, 2*pi, 360);

x = r*cos(t) + O(1);
y = r*sin(t) + O(2);

plot_handle = plot(x,y,linespec);
end

% Plots the state of the network at iteration s
function plot_state( simulation_parameters, agents, agent_state, s, colors )
	% Number of agents
	N = simulation_parameters.N;
	% Find sensing patterns
	sensing = cell([1 N]);
	for i=1:N
		if agent_state{i}.relaxed_sensing_quality(s) == 0
			sensing{i} = agents{i}.base_guaranteed_sensing;
		elseif agent_state{i}.relaxed_sensing_quality(s) == 1
			sensing{i} = agents{i}.base_total_sensing;
		end
		sensing{i} = rot( sensing{i}, agent_state{i}.yaw(s) ) + ...
		[agent_state{i}.x(s) ; agent_state{i}.y(s)];
	end
	% Find cells
	W = partition( simulation_parameters.region, sensing );
	hold on
	axis equal
	axis off
	axis(simulation_parameters.axis);
	plot_poly( simulation_parameters.region, 'k' );
	for i=1:N
		% Show cell
		if ~isempty(W{i})
			if nargin == 5
				fill_nan( W{i}(1,:), W{i}(2,:), 'r', 1, colors(i,:) );
			else
				fill_nan( W{i}(1,:), W{i}(2,:), 'r' );
			end
		end
		% Show sensing
		plot_poly( sensing{i}, 'k' );
		% Show random real sensing
		real_sensing = rot( agents{i}.base_sensing, ...
		agent_state{i}.yaw(s) + agents{i}.random_attitude ) + ...
		[agent_state{i}.x(s) ; agent_state{i}.y(s)] + agents{i}.random_position;
% 		plot_poly( real_sensing, 'k--' );
		% Show reported state and uncertainty
		plot_uncertainty( [agent_state{i}.x(s) ; agent_state{i}.y(s)],...
		agents{i}.position_uncertainty,...
		agent_state{i}.yaw(s), agents{i}.attitude_uncertainty, 'k' );
		% Show agent ID
		text(agent_state{i}.x(s) - 0.5*agents{i}.position_uncertainty, ...
		agent_state{i}.y(s) - 2*agents{i}.position_uncertainty, ...
		sprintf('%d',agents{i}.ID), 'FontSize', 15);
	end
end

% Finds the area of a polygon with holes
% polygon contours are Nan delimited
function area = polyarea_nan(x, y)
	% The outer contour is the last NaN delimited segment
	nan_indices = find(isnan( x ));
	indices = [ 0 nan_indices length(x)+1 ];
	area = 0;

	if ~isempty(nan_indices)
		for i=1:length(nan_indices)+1

			tempx = x( indices(i)+1 : indices(i+1)-1 );
			tempy = y( indices(i)+1 : indices(i+1)-1 );
			if ispolycw(tempx, tempy)
				% external contour
				area = area + polyarea(tempx, tempy);
			else
				% internal contour
				area = area - polyarea(tempx, tempy);
			end

		end
	else
		area = polyarea(x, y);
	end
end