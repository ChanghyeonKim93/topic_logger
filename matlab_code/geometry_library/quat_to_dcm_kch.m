function dcm = quat_to_dcm_kch(quat)
% Author : Changhyeon Kim, ICSL, Seoul National University.
% Date   : 2018-08-23
% Funtionality : calculate direct cosine matrix (DCM) using quaternion.
% Input  : 4-d quaternion
% Output : 3x3 DCM

if(size(quat,1) < size(quat,2))
    quat = quat.'; % vectorize
end

quat_w = quat(1,1);
quat_v = quat(2:4,1);


w=quat(1); 
x=quat(2);
y=quat(3);
z=quat(4);

dcm = zeros(3);
dcm(1,1) = w*w + x*x - y*y - z*z;
dcm(1,2) = 2*(x*y - w*z);
dcm(1,3) = 2*(x*z + w*y);
dcm(2,1) = 2*(x*y + w*z);
dcm(2,2) = w*w - x*x + y*y - z*z;
dcm(2,3) = 2*(y*z - w*x);
dcm(3,1) = 2*(x*z - w*y);
dcm(3,2) = 2*(y*z + w*x);
dcm(3,3) = w*w - x*x - y*y + z*z;

% dcm = (quat_w*quat_w -
% quat_v.'*quat_v)*eye(3)+2*quat_v*quat_v.'+2*quat_w*vect_skew_kch(quat_v);
% equivalent description
end