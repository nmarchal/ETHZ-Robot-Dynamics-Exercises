function J_R = jointToRotJac(q)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector orientation which maps joint
  % velocities to end-effector angular velocities in I frame.
%   Variablke with all the rotation matrices
  
  C = [] ; J_R = [] ;
  C = [ tranformationToRotation(getTransformI0()) ...
      tranformationToRotation(jointToTransform01(q)) ...
      tranformationToRotation(jointToTransform12(q)) ...
      tranformationToRotation(jointToTransform23(q)) ...
      tranformationToRotation(jointToTransform34(q)) ...
      tranformationToRotation(jointToTransform45(q)) ...
      tranformationToRotation(jointToTransform56(q)) ...
      tranformationToRotation(getTransform6E())] ;

  %first colum is rotx, 2nd is rot y and last rotz
  rotDirection = [1 0 0 ; 0 1 0 ; 0 0 1] ;
  rot = [3 2 2 1 2 1] ;
  for i = 1:6
      rotationMatrix = eye(3) ;
      for j = 1:i 
          index = 3*(j-1) + 1 ;
          rotationMatrix = rotationMatrix * C(:,index:index+2) ;
      end
      % Compute the rotational jacobian.
      J_R(:,i) =  rotationMatrix * rotDirection(:,rot(i)) ;
  end

end
