function J_P = jointToPosJac(q)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector translation which maps joint
  % velocities to end-effector linear velocities in I frame.

  C = [] ; J_P = [] ;
%   All rotation matrices
  C = [tranformationToRotation(jointToTransform01(q)) ...
      tranformationToRotation(jointToTransform12(q)) ...
      tranformationToRotation(jointToTransform23(q)) ...
      tranformationToRotation(jointToTransform34(q)) ...
      tranformationToRotation(jointToTransform45(q)) ...
      tranformationToRotation(jointToTransform56(q)) ] ;
  
%   All tranformation matrices
  T = [ getTransformI0_solution() ...
      jointToTransform01(q) ...
      jointToTransform12(q) ...
      jointToTransform23(q) ...
      jointToTransform34(q) ...
      jointToTransform45(q) ...
      jointToTransform56(q) ] ;
  
% Position of all end effector
    pos = [ tranformationToPosition(jointToTransform01(q)) ...
      tranformationToPosition(jointToTransform12(q)) ...
      tranformationToPosition(jointToTransform23(q)) ...
      tranformationToPosition(jointToTransform34(q)) ...
      tranformationToPosition(jointToTransform45(q)) ...
      tranformationToPosition(jointToTransform56(q)) ] ;
    
  % end effector position
  endPosition = jointToPosition(q) ;
  
  %first colum is rotx, 2nd is rot y and last rotz
  rotDirection = [1 0 0 ; 0 1 0 ; 0 0 1] ;
%   rotation of the ABB robot
  rot = [3 2 2 1 2 1] ;
   
  for i = 1:6
      rotationMatrix = eye(3) ;
      transformationMatrix = eye(4) ;
      jointPos = pos(:,i) ;
      for j = 1:i 
          indexC = 3*(j-1) + 1 ;
          indexT = 4*(j-1) + 1 ;
          rotationMatrix = rotationMatrix * C(:,indexC:indexC+2) ;
          transformationMatrix = transformationMatrix *  T(:,indexT:indexT+3) ;
      end      
      nVect = rotationMatrix * rotDirection(:,rot(i)) ;
      % ca aurait ete plus simple de prendre le vecteur de la
      % transformation matrix appropriée
      jointPos = transformationMatrix * [jointPos ; 1] ;
      jointPos = jointPos(1:3,:) ;
      %
      tVect = endPosition-jointPos ;
      J_P(:,i) =  cross(nVect, tVect) ;
  end
end