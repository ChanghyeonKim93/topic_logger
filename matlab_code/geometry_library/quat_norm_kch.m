function norm_value = quat_norm_kch(quat)
% Author : Changhyeon Kim, ICSL, Seoul National University.
% Date   : 2018-08-23
% Funtionality : calculate 4-d quaternion norm.
% Input  : 4-d quaternion
% Output : norm value

if(size(quat,1) < size(quat,2))
    quat = quat.'; % vectorize
end
norm_value = sqrt( quat(1,1)*quat(1,1) + quat(2:4,1).'*quat(2:4,1) );
end