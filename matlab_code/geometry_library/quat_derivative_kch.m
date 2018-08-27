function dquat = quat_derivative_kch(quat, vect)
% Author : Changhyeon Kim, ICSL, Seoul National University.
% Date   : 2018-08-23
% Funtionality : calculate 4-d quaternion derivative.
% Input  : (1) 4-d quaternion, (2) 3-d angular vel. vector (local frame)
% Output : time derivative of 4-d quaternion

if(size(quat,1) < size(quat,2))
    quat = quat.'; % vectorize
end
if(size(vect,1) < size(vect,2))
    vect = vect.';
end

% Omega = [0,-vect.';vect,-vect_skew_kch(vect)];
Omega = quat_right_prod_mat_kch([0;vect]);
dquat = 0.5*Omega*quat;

end