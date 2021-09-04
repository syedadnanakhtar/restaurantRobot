function [finalScore,outputCoordinates] = Astar2(map,x,y,destination)
%Calculates coordinates for checkpoints robot needs to pass to reach
%destination. x = x coordinate at start of robot, y = y coordinate at start
%of robot. destination is location number of the destination.


x = 20*x;
y = 20*y;

pointLocations = map{1};

% Connections {node number}(connected node numbers;weights)
connections = map{2};

%% Find Nearest Node at start

dToPoint = zeros(length(pointLocations),1);

for i = 1:length(pointLocations)
    dToPoint(i) = (pointLocations(i,1)-x)^2+(pointLocations(i,2)-y)^2;
end
[~,startCheckpoint]=min(dToPoint);

%% Calculate Heuristics for A* algorithm
% Euclidean distance is used as heuristic
xdest = pointLocations(destination,1);
ydest = pointLocations(destination,2);
dToDestination = zeros(length(pointLocations),1);

for i = 1:length(pointLocations)
    dToDestination(i) = sqrt((pointLocations(i,1)-xdest)^2+(pointLocations(i,2)-ydest)^2);
end
A = 35/max(dToDestination) ; % A star heuristic scaling coefficiënt
AHeur = A*dToDestination;


%% Do Graph Search


%create array for graph search {cell number,property}
% Property => 1:checked yet ([] not seen 1: checked all branches 2:is
% connected to a previously checked node)
% 2:shortest distance 
% 3:shortest path to node
% distance
dijkArray = cell(length(pointLocations),3);
dijkArray{startCheckpoint,1} = 1;
dijkArray{startCheckpoint,2} = 0;
dijkArray{startCheckpoint,3} = [];

currentNode = startCheckpoint;
stopSearching = 0;


while(stopSearching < 2)
%find distance to next checkpoints on graph and update dijkArray
for j = 1:length(connections{currentNode}(1,:))
   newDistance = 0;
   newPath     = 0;             %Reset newDistance and newNode
    if isempty(dijkArray{connections{currentNode}(1,j),1})  %Check if the connected node is already done
        
        newDistance = dijkArray{currentNode,2}+connections{currentNode}(2,j)+AHeur(connections{currentNode}(1,j)); %Calculate overall distance from start
        newPath = [dijkArray{currentNode,3};connections{currentNode}(1,j)];                     %Create path to that point   
        
        if isempty(dijkArray{connections{currentNode}(1,j),2})                %fill in distance and create path if it's not yet created       
            dijkArray{connections{currentNode}(1,j),2}= newDistance;  
            dijkArray{connections{currentNode}(1,j),3}= newPath;
            dijkArray{connections{currentNode}(1,j),1}= 2;                    % 2 means make available to be checked
        else 
            if newDistance < dijkArray{connections{currentNode}(1,j),2}       %Check if calculated distance is smaller than current
              dijkArray{connections{currentNode}(1,j),2} = newDistance;       %overwrite if it's smaller 
              dijkArray{connections{currentNode}(1,j),3}= newPath;
            end
        end
         
    %connected node number = connections{currentNode}(1,j)    
    %connected node distance = connections{currentNode}(2,j)            
    end    
    
end
%Set Current Node to checked
dijkArray{currentNode,1} = 1;

%Look for next node to check
nodeCheck = 1;
stopSearching = 0;
while(stopSearching == 0)
    if nodeCheck > length(dijkArray)
    stopSearching = 2;
    elseif dijkArray{nodeCheck,1} == 2
    currentNode = nodeCheck;   
    stopSearching = 1;
    end
    nodeCheck = nodeCheck + 1;
end
end

checkpoints = [startCheckpoint;dijkArray{destination,3}];
finalScore = dijkArray{destination,2};
outputCoordinates = zeros(length(checkpoints),2);
%Convert checkpoint-numbers back to coordinates
for a = 1:length(checkpoints)
    outputCoordinates(a,:) = 0.05*pointLocations(checkpoints(a),:);
end
end



