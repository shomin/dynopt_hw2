function [C, Cd, Cdd] = robot_spline(th_0,thd_0,thdd_0,th_f,thd_f,thdd_f, tf)

H = [1 0 0 -10 15 -6;...
    0 1 0 -6 8 -3;...
    0 0 .5 -1.5 1.5 -.5;...
    0 0 0 10 -15 6;...
    0 0 0 -4 7 -3;...
    0 0 0 .5 -1 .5];

H = H(:,6:-1:1); % flip for MATLAB polynomial convention

t_pow = 1./[tf^5 tf^4 tf^3 tf^2 tf 1];
Ht = H.*repmat(t_pow,6,1);

C = th_0*Ht(1,:) + thd_0*Ht(2,:) + thdd_0*Ht(3,:) + ...
    th_f*Ht(4,:) + thd_f*Ht(5,:) + thdd_f*Ht(6,:);

Cd = C.*repmat(  [ 5  4 3 2 1 0], length(th_0), 1);
Cdd = C.*repmat( [20 12 6 2 0 0], length(th_0), 1);


