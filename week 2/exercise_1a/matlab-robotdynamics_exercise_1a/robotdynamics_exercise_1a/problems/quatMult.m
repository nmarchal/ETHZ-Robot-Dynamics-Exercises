function q_AC = quatMult(q,p)
  % Input: two quaternions to be multiplied
  % Output: output of the multiplication
  
  q_im = q(2:4,1) ;
  q_re = q(1,1) ;
  q_AC = [q_re -q_im' ;
          q_im (q_re*eye(3) + crossProdMat(q_im)) ] * p ;
end