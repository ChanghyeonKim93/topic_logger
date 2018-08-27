function quat_prod= quat_prod_kch(quat1, quat2)
% Author : Changhyeon Kim, ICSL, Seoul National University.
% Date   : 2018-08-23
% Funtionality : product (multiplication) two 4-d quaternions.
% Input  : (1) 4-d quaternion1, (2) 4-d quaternion2.
% Output : 4-d quaternion (product)

if(size(quat1,1) < size(quat1,2))
    quat1 = quat1.'; % vectorize
end
if(size(quat2,1) < size(quat2,2))
    quat2 = quat2.'; % vectorize
end

quat_prod = [quat1(1,1)*quat2(1,1)-quat1(2:4,1).'*quat2(2:4,1);quat1(1,1)*quat2(2:4,1)+quat2(1,1)*quat1(2:4,1)-vect_cross_kch(quat1(2:4,1),quat2(2:4,1))];

end