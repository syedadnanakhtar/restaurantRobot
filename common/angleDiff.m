function rval = angleDiff(angle2, angle1)
% ANGLEDIFF Computes the difference between two angles.
% The input angles have to be given in rad. The output is in the range [-pi, pi]
    angle1 = mod(angle1, 2*pi);
    angle2 = mod(angle2, 2*pi);
    rval = mod((angle2-angle1)+pi, 2*pi) -pi;
end