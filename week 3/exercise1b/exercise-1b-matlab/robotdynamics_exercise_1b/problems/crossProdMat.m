function mat = crossProdMat(v) 
% takes a vector V and outputs the matrix simulation a cross product
x = v(1) ; y = v(2) ; z = v(3) ;
mat = -[0 z -y ; -z 0 x ; y -x 0]; 
end