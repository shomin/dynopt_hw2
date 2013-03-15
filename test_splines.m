
th_0 = zeros(28,1);
thd_0 = zeros(28,1);
thdd_0 = zeros(28,1);

th_f = .4.*ones(28,1);
thd_f = zeros(28,1);
thdd_f = zeros(28,1);


tf = .5; %half second trajectory
[C, Cd, Cdd] = robot_spline(th_0,thd_0,thdd_0,th_f,thd_f,thdd_f, tf);

speed = .5; %play at half speed
play_poly(C,tf,speed);