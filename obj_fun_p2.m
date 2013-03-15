function f = obj_fun_p2(X,x_start,tf,n)

th_0 = x_start;
thd_0 = zeros(28,1);
thdd_0 = zeros(28,1);

th_f = zeros(28,1);
thd_f = zeros(28,1);
thdd_f = zeros(28,1);


thdd_0 = X(1:28);
thdd_f = X(29:56);
% thdd_f = X(57:84);



% tf = .5; %half second trajectory
[C, Cd, Cdd] = robot_spline(th_0,thd_0,thdd_0,th_f,thd_f,thdd_f, tf);


r = atlas_data;
vel_lims = cell2mat({r.velocity_limits}');
torq_lims = cell2mat({r.torque_limits}');
vel_lims = vel_lims(2:end,:);
torq_lims = torq_lims(2:end,:);

sum_t_viol = 0;
sum_v_viol = 0;
sum_com_viol = 0;
sum_torqs = 0;

sum_lfoot = 0;
sum_rfoot = 0;

% n = 20; %samples along trajectory
ts = linspace(0,tf,n);
for t = ts
    t_pow = [t^5 t^4 t^3 t^2 t 1]';
    th   =   C*t_pow;
    thd  =  Cd*t_pow;
    thdd = Cdd*t_pow;

    [torqs, pos, lf,lt,rf,rt,r_or,l_or,com] = atlas(th,thd,thdd);
    torqs = torqs(2:end)';
    
    sum_torqs = sum_torqs + sum(abs(torqs));
    
    t_viol = [-(torq_lims(:,2) - torqs); (torq_lims(:,1) - torqs)];
    sum_t_viol = sum_t_viol + sum(t_viol(t_viol>0));
    
    v_viol = [-(vel_lims(:,2) - thd); (vel_lims(:,1) - thd)];    
    sum_v_viol = sum_v_viol + sum(v_viol(v_viol>0));

    com_viol = [com(1) - (.262/2), -com(1) - (.262/2),...
        com(2) - (.302/2), -com(2) - (.302/2)];
    sum_com_viol = sum_com_viol + sum(com_viol(com_viol>0));
    
    l_foot_error = sum(([-0.0484  0.0890 0.0811]-pos(:,11)').^2);
    r_foot_error = sum(([-0.0484 -0.0890 0.0811]-pos(:,17)').^2);
    
    sum_lfoot = sum_lfoot + l_foot_error;
    sum_rfoot = sum_rfoot + r_foot_error;

%     th
%     com
end




%angle deviation for feet
l_ang = 1-((rot2quat(r_or)*[1 0 0 0]')^2);
r_ang = 1-((rot2quat(l_or)*[1 0 0 0]')^2);


% f = wr_sq_error;
lam = [10 10 1 1 .01 .0001 .01 .001]; %weights
f = lam*[sum_lfoot sum_rfoot l_ang r_ang...
    sum_t_viol sum_v_viol sum_com_viol sum_torqs]';

end