function vect_cross= vect_cross_kch(vect1, vect2)
% Author : Changhyeon Kim, ICSL, Seoul National University.
% Date   : 2018-08-23
% Funtionality : cross product two 3-d vectors.
% Input  : (1) 3-d vector1, (2) 3-d vector2.
% Output : 3-d vector (cross product)

if(size(vect1,1) < size(vect1,2))
    vect1 = vect1.'; % vectorize
end
if(size(vect2,1) < size(vect2,2))
    vect2 = vect2.'; % vectorize
end

vect_cross = [vect1(2)*vect2(3)-vect2(2)*vect1(3); vect1(3)*vect2(1)-vect2(3)*vect1(1); vect1(1)*vect2(2)-vect2(1)*vect1(2)];
end