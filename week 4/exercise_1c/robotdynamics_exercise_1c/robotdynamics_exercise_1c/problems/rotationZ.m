function C = rotationZ( phi )
%Rotation matrix for a rotation around the Z axis

C = [   cos(phi)           -sin(phi)          0 ; ...
        sin(phi)           cos(phi)           0 ; ...
        0                  0                  1 ] ;
end