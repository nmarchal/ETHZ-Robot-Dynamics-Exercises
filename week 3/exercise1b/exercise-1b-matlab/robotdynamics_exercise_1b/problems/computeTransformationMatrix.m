function [ T ] = computeTransformationMatrix( C, vector )
%Compute the transformation matrix knowng the rotation matrix and
%translation vector
T = [ C vector ;
    zeros(1,3) 1] ;
end

