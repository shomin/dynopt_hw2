function q = rot2quat(R)

%from http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion

t = R(1,1)+R(2,2)+R(3,3); % (trace of Q)
r = sqrt(1+t);
q(1) = 0.5*r;
q(2) = abs(0.5*sqrt(1+R(1,1)-R(2,2)-R(3,3))) * sign(R(3,2)-R(2,3));
q(3) = abs(0.5*sqrt(1-R(1,1)+R(2,2)-R(3,3))) * sign(R(1,3)-R(3,1));
q(4) = abs(0.5*sqrt(1-R(1,1)-R(2,2)+R(3,3))) * sign(R(2,1)-R(1,2));