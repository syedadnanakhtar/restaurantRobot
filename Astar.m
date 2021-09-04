function [path] = Astar(map,x,y,destination)
[score1,path1] = Astar1(map,x,y,destination)
[score2,path2] = Astar2(map,x,y,destination)

if score2 < score1
    path = path1;
else
    path = path2;
end

end