function left_mat = quat_left_prod_mat_kch(quat)
% Author : Changhyeon Kim, ICSL, Seoul National University.
% Date   : 2018-08-23
% Funtionality : convert 4-d quaternion into left product matrix (4x4).
% Input  : 3-d vector
% Output : left product matrix (4x4).

if(size(quat,1) < size(quat,2))
    quat = quat.'; % vectorize
end

skew_mat = skew_kch(quat(2:4,1));
quat_v = quat(2:4,1);

left_mat = quat(1,1)*eye(4) + [0, -quat(2:4,1).'; quat(2:4,1), skew_mat];


end