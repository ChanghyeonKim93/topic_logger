function quat_inv = quat_inv_kch(quat)
% Author : Changhyeon Kim, ICSL, Seoul National University.
% Date   : 2018-08-23
% Funtionality : calculate 4-d quaternion inverse.
% Input  : 4-d quaternion
% Output : 4-d quaternion (inverse)

if(size(quat,1) < size(quat,2))
    quat = quat.'; % vectorize
end

quat_inv = quat_conj_kch(quat)/( quat_norm_kch(quat)^2 );

end