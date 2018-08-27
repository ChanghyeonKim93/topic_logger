function right_mat = quat_right_prod_mat_kch(quat)
% Author : Changhyeon Kim, ICSL, Seoul National University.
% Date   : 2018-08-23
% Funtionality : convert 4-d quaternion into right product matrix (4x4).
% Input  : 3-d vector
% Output : right product matrix (4x4).

if(size(quat,1) < size(quat,2))
    quat = quat.'; % vectorize
end

quat_v = quat(2:4,1);
skew_mat = vect_skew_kch(quat_v);

right_mat = quat(1,1)*eye(4,4) + [0, -quat_v.'; quat_v, -skew_mat];


end