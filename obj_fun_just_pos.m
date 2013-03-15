function f = obj_fun_just_pos(X,goal,tf,n)


th_0 = zeros(28,1);
thd_0 = zeros(28,1);
thdd_0 = zeros(28,1);

thd_f = zeros(28,1);
thdd_f = zeros(28,1);


th_f = X(1:28);



% tf = .5; %half second trajectory
[C, Cd, Cdd] = robot_spline(th_0,thd_0,thdd_0,th_f,thd_f,thdd_f, tf);


% n = 20; %samples along trajectory



t = tf;
t_pow = [t^5 t^4 t^3 t^2 t 1]';
th   =   C*t_pow;
thd  =  Cd*t_pow;
thdd = Cdd*t_pow;


[torqs, pos, lf,lt,rf,rt,r_or] = atlas(th,thd,thdd);


wrist = pos(:,end);


wr_sq_error = sum((goal - wrist).^2);


f = wr_sq_error;

end