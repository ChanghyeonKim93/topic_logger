function euler= quat_to_euler(quat)
% Author : Changhyeon Kim, ICSL, Seoul National University.
% Date   : 2018-08-23
% Funtionality : convert 4-d quaternion to Euler angle (especially, ZYX).
% Input  : 4-d quaternion
% Output : 3-d Euler angle vector

if(size(quat,1) < size(quat,2))
    if(size(quat,1) ~= 4)
        quat = quat.'; % vectorize
    end
end
if(size(quat,2)>2)
    euler = zeros(3,length(quat));
    for i=1:length(quat)
        
        w = quat(1,i);
        x = quat(2,i);
        y = quat(3,i);
        z = quat(4,i);
        
        euler(:,i) = [atan2(2*(w*x+y*z),1-2*(x*x+y*y));asin(2*(w*y-z*x));atan2(2*(w*z+x*y),1-2*(y*y+z*z))];
    end
else
    w = quat(1);
    x = quat(2);
    y = quat(3);
    z = quat(4);
    
    euler = [atan2(2*(w*x+y*z),1-2*(x*x+y*y));asin(2*(w*y-z*x));atan2(2*(w*z+x*y),1-2*(y*y+z*z))];
end

end