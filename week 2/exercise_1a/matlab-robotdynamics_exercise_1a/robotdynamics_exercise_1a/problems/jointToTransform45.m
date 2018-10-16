function T45 = jointToTransform45(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 5 to frame 4. T_45
  C = rotationY(q(5)) ;
  vector = [0.168 ; 0 ; 0] ;
  T45 = computeTransformationMatrix(C, vector) ;
end

