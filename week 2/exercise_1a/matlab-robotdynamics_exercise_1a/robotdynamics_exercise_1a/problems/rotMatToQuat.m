function q = rotMatToQuat(R)
  % Input: rotation matrix
  % Output: corresponding quaternion [w x y z]
  
  q1 = sqrt(1 + R(1,1) + R(2,2) + R(3,3)) ;
  q2 = sign(R(3,2)-R(2,3)) * sqrt(1 + R(1,1) - R(2,2) - R(3,3)) ;
  q3 = sign(R(1,3)-R(3,1)) * sqrt(1 - R(1,1) + R(2,2) - R(3,3)) ;
  q4 = sign(R(2,1)-R(1,2)) * sqrt(1 - R(1,1) - R(2,2) + R(3,3)) ;
  
  q=1/2*[q1 ; q2 ; q3 ; q4] ;
end