function quat_conj = quat_conj_kch(quat)
% Author : Changhyeon Kim, ICSL, Seoul National University.
% Date   : 2018-08-23
% Funtionality : calculate 4-d quaternion conjugate.
% Input  : 4-d quaternion
% Output : 4-d quaternion (conjugate)

if(size(quat,1) < size(quat,2))
    quat = quat.'; % vectorize
end

quat_conj =[quat(1,1);-quat(2:4,1)];

end