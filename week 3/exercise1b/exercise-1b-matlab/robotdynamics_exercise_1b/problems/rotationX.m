function C = rotationX( phi )
%Rotation matrix for a rotation around the X axis

C = [   1           0           0 ; ...
        0           cos(phi)    -sin(phi) ; ...
        0           sin(phi)    cos(phi) ] ;
end