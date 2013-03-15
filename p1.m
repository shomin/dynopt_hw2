


goal = [-0.4 -.4 .8]';
tf = 1;
n = 20;
f_obj = @(x)obj_fun_p1(x,goal,tf,n);
% f_obj = @(x)obj_fun_just_pos(x,goal,tf,n);


% x0 = zeros(28,1);
% x0 = zeros(56,1);
x0 = zeros(84,1);

r = atlas_data();
ang_limits = cell2mat({r.angle_limits}');
lb = ang_limits(2:end,1);
ub = ang_limits(2:end,2);



X = fmincon(f_obj,x0,[],[],[],[],lb,ub);%,c_fun,options);
% X = fminunc(f_obj,x0);





%%

th_0 = zeros(28,1);
thd_0 = zeros(28,1);
thdd_0 = zeros(28,1);

th_f = zeros(28,1);
thd_f = zeros(28,1);
thdd_f = zeros(28,1);


th_f = X(1:28);
thdd_0 = X(29:56);
thdd_f = X(57:84);


[C, Cd, Cdd] = robot_spline(th_0,thd_0,thdd_0,th_f,thd_f,thdd_f, tf);


r = init_robot();
mopts = {'.','Color',[0.3 0.5 0.9], 'MarkerSize', 30};
plot3(goal(1),goal(2),goal(3),mopts{:});

lopts ={'k-','LineWidth',2};
plot3([1 1 -1 -1 1].*(.262/2), [1 -1 -1 1 1].*(.302/2), [0 0 0 0 0],lopts{:});
plot3([1 1 -1 -1 1].*(.262/2), [1 -1 -1 1 1].*((.302/2)-.124), [0 0 0 0 0],lopts{:});

drawnow;


speed = .5; %play at half speed
play_poly(C,tf,speed,r);