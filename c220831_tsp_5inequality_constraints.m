%%
clear all;
close all;
clc;


%% Generate Stops
load('georgia.mat');
nStops = 100;               % # stops
stopsLon = zeros(nStops,1); % Allocate x-coordinates of nStops
stopsLat = stopsLon;        % Allocate y-coordinates
n = 1;
while (n <= nStops)
    display(['inpolygon testing... ' num2str(n) '/' num2str(nStops)]);
    xp = sqrt(max(lon)-min(lon)) * randn + median(lon);
    yp = sqrt(max(lat)-min(lat)) * randn + median(lat);
    if inpolygon(xp,yp,lon,lat) % Test if inside the border
        stopsLon(n) = xp;
        stopsLat(n) = yp;
        n = n+1;
    end
end


%% Calculate Distances Between Points
% Generate all the trips, meaning all pairs of stops
idxs = nchoosek(1:nStops,2);

% Calculate all the trip distances
dist = hypot(stopsLat(idxs(:,1)) - stopsLat(idxs(:,2)), ...
             stopsLon(idxs(:,1)) - stopsLon(idxs(:,2)));
lendist = length(dist);


%% Create Graph and Draw Map
G = graph(idxs(:,1),idxs(:,2));

figure
hGraph = plot(G,'XData',stopsLon,'YData',stopsLat,'LineStyle','none','NodeLabel',{});
box on
grid on
hold on
xlabel('Longitude')
ylabel('Latitude')
title('TSP solved on the State of Georgia USA')
ylim([30,35.1])
% Draw the outside border
plot(lon,lat,'k','Linewidth',2)
hold off


%% Create Variables and Problem
% Create an optimization problem with binary optimization variables representing the potential trips
tsp = optimproblem;
trips = optimvar('trips',lendist,1,'Type','integer','LowerBound',0,'UpperBound',1);

% Include the objective function in the problem
tsp.Objective = dist'*trips;


%% Constraints
% For each stop, create the constraint that the sum of trips for that stop equals two
constr2trips = optimconstr(nStops,1);
for stop = 1:nStops
    display(['Constraint creating at each stop... ' num2str(stop) '/' num2str(nStops)]);
    whichIdxs = outedges(G,stop); % Identify trips associated with the stop
    constr2trips(stop) = sum(trips(whichIdxs)) == 2;
end
tsp.Constraints.constr2trips = constr2trips;


%% Solve Initial Problem
% To suppress iterative output, turn off the default display
opts = optimoptions('intlinprog','Display','off');
tspsol = solve(tsp,'options',opts)


%% Visualize Solution
% Create a new graph with the solution trips as edges
tspsol.trips = logical(round(tspsol.trips));
Gsol = graph(idxs(tspsol.trips,1),idxs(tspsol.trips,2),[],numnodes(G));
% Gsol = graph(idxs(tspsol.trips,1),idxs(tspsol.trips,2)); % Also works in most cases

% Overlay the new graph on the existing plot and highlight its edges
hold on
highlight(hGraph,Gsol,'LineStyle','-')
% title('Solution with Subtours')


%% Subtour Constraints
% Detect the subtours by identifying the connected components in Gsol
tourIdxs = conncomp(Gsol);
numtours = max(tourIdxs); % Number of subtours
fprintf('# of subtours: %d\n',numtours);

% Eepeatedly call the solver, until just one subtour remains
k = 1;  % Index of added constraints for subtours
while numtours > 1 % Repeat until there is just one subtour
    % Add the subtour constraints
    for ii = 1:numtours
        inSubTour = (tourIdxs == ii); % Edges in current subtour
        a = all(inSubTour(idxs),2); % Complete graph indices with both ends in subtour
        constrname = "subtourconstr" + num2str(k);
        tsp.Constraints.(constrname) = sum(trips(a)) <= (nnz(inSubTour) - 1);
        k = k + 1;        
    end
    
    % Try to optimize again
    [tspsol,fval,exitflag,output] = solve(tsp,'options',opts);
    tspsol.trips = logical(round(tspsol.trips));
    Gsol = graph(idxs(tspsol.trips,1),idxs(tspsol.trips,2),[],numnodes(G));
    % Gsol = graph(idxs(tspsol.trips,1),idxs(tspsol.trips,2)); % Also works in most cases
    
    % Plot new solution
    hGraph.LineStyle = 'none'; % Remove the previous highlighted path
    highlight(hGraph,Gsol,'LineStyle','-')
    drawnow

    % How many subtours this time?
    tourIdxs = conncomp(Gsol);
    numtours = max(tourIdxs); % Number of subtours
    fprintf('# of subtours: %d\n',numtours)    
end

