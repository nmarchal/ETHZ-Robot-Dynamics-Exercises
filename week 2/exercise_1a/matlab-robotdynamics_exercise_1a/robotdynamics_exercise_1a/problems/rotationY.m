function C = rotationY( phi )
%Rotation matrix for a rotation around the Y axis

C = [   cos(phi)           0           sin(phi) ; ...
        0                  1           0 ; ...
        -sin(phi)          0           cos(phi) ] ;
end