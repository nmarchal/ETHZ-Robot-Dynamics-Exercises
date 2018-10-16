function T34 = jointToTransform34(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 4 to frame 3. T_34
  C = rotationX(q(4)) ;
  vector = [0.134 ; 0 ; 0.07] ;
  T34 = computeTransformationMatrix(C, vector) ;
end

