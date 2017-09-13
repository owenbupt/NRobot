## File Formats
Below is a description of the file formats used when data is imported and exported. All files used are simple ASCII text files.

#### Contour
```
<Number of vertices>
<Hole flag (omitted by default)>
<Open flag (omitted by default)>
for each contour vertex
    <x coordinate> <y coordinate>
```

#### Polygon
```
<Number of contours>
for each contour
    <Number of vertices>
    <Hole flag (omitted by default)>
    <Open flag (omitted by default)>
    for each contour vertex
        <x coordinate> <y coordinate>
```

#### Simulation Parameters
Filename: `sim_YYYYMMDD_HHMMSS_parameters.txt`. Saved at the end of the simulation.
```
<Number of agents>
<Simulation duration>
<Simulation time-step>
<Simulation iterations>
<Elapsed simulation time>
<Average iteration time>
<Objective function values (a line with <Simulation iterations> values)>
<Objective function threshold>
<Objective function average window>
<Region (using Polygon format)>
```

#### Agent Parameters
Filename: `sim_YYYYMMDD_HHMMSS_agent_X_parameters.txt` where X is the agent ID. Saved at the beginning of the simulation.
```
<Agent ID>
<Initial position>
<Initial attitude>
<Initial translational velocity>
<Initial rotational velocity>
<Sensing radius>
<Communication radius>
<Position uncertainty>
<Attitude uncertainty>
<Initial relaxed sensing quality>
<Agent dynamics>
<Agent time-step>
<Partitioning scheme>
<Control scheme>
<Collision avoidance scheme>
<Control input gains (a line with a number of values dependent on <Agent dynamics>)>
<Base sensing (using Polygon format)>
<Base guaranteed sensing (using Polygon format)>
<Base relaxed sensing (using Polygon format)>
<Base total sensing (using Polygon format)>
```

#### Agent State
This file is organized in columns. Each column contains < Simulation iterations > elements. Each column is shown in a different line for clarity below.
Filename: `sim_YYYYMMDD_HHMMSS_agent_X_state.txt` where X is the agent ID. Saved at the end of the simulation.
```
<Iteration number>
<Position x>
<Position y>
<Position z>
<Attitude roll>
<Attitude pitch>
<Attitude yaw>
<Velocity x>
<Velocity y>
<Velocity z>
<Velocity roll>
<Velocity pitch>
<Velocity yaw>
<Relaxed sensing quality>
<Connected with neighbor 1>
<Connected with neighbor 2>
.
.
.
<Connected with neighbor N>
<Control input 1>
<Control input 2>
.
.
.
<Control input m (m is dependent on <Agent dynamics>)>
```
