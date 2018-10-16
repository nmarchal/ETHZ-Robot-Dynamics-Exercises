function J_P = jointToPosJac(q)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector translation which maps joint
  % velocities to end-effector linear velocities in I frame.

%   Individual transformation matrices 
  T01 = jointToTransform01(q) ;
  T12 = jointToTransform12(q) ;
  T23 = jointToTransform23(q) ;
  T34 = jointToTransform34(q) ;
  T45 = jointToTransform45(q) ;
  T56 = jointToTransform56(q) ;
  
%   Total trnasformation matrices 
  T02 = T01*T12 ;
  T03 = T02*T23 ;
  T04 = T03*T34 ;
  T05 = T04*T45 ;
  T06 = T05*T56 ;
  
%   Rotation matrices
  C01 = T01(1:3,1:3) ;
  C02 = T02(1:3,1:3) ;
  C03 = T03(1:3,1:3) ;
  C04 = T04(1:3,1:3) ;
  C05 = T05(1:3,1:3) ;
  C06 = T06(1:3,1:3) ;

%   for transformation form 
  x = [1;0;0] ;
  y = [0;1;0] ;
  z = [0;0;1] ;
  
%   all the axis in inertial frame 
  n1 = C01*z ; 
  n2 = C02*y ; 
  n3 = C03*y ; 
  n4 = C04*x ; 
  n5 = C05*y ; 
  n6 = C06*x ; 
  
  %   All position in inertial frame
  r1 = T01(1:3,4) ;
  r2 = T02(1:3,4) ;
  r3 = T03(1:3,4) ;
  r4 = T04(1:3,4) ;
  r5 = T05(1:3,4) ;
  r6 = T06(1:3,4) ;
  
%   Difference in position (operation possible because all in inertial frame)
  r1 = r6 - T01(1:3,4) ;
  r2 = r6 - T02(1:3,4) ;
  r3 = r6 - T03(1:3,4) ;
  r4 = r6 - T04(1:3,4) ;
  r5 = r6 - T05(1:3,4) ;
  r6 = r6 - T06(1:3,4) ;

%   Final solution for position Jacobian 
  J_P = [cross(n1,r1) cross(n2,r2) cross(n3,r3) ... 
        cross(n4,r4) cross(n5,r5) cross(n6,r6)] ;  
end