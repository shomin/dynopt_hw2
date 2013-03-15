load p1_good.mat

bad_data = data(5:6,:);
data = data([1:4,7:end],:);


vid = 1;
if(vid)
 vidObj = VideoWriter('dynopt_hw2_movbad.avi');
 vidObj.Quality = 100;
 vidObj.FrameRate = 30;
 open(vidObj);
end

data = bad_data;

for i = 1:size(data,1)
i
	th_0 = zeros(28,1);
	thd_0 = zeros(28,1);
	thdd_0 = zeros(28,1);
	
	th_f = zeros(28,1);
	thd_f = zeros(28,1);
	thdd_f = zeros(28,1);
	
	
	goal = data{i,1};
	tf = data{i,2};
	n = data{i,3};
	X = data{i,4};
	
	th_f = X(1:28);
	thdd_0 = X(29:56);
	thdd_f = X(57:84);
	
	
	[C, Cd, Cdd] = robot_spline(th_0,thd_0,thdd_0,th_f,thd_f,thdd_f, tf);
	
	clf;
	r = init_robot();	

	mopts = {'.','Color',[0.3 0.5 0.9], 'MarkerSize', 30};
	plot3(goal(1),goal(2),goal(3),mopts{:});
	
	lopts ={'k-','LineWidth',2};
	plot3([1 1 -1 -1 1].*(.262/2), [1 -1 -1 1 1].*(.302/2), [0 0 0 0 0],lopts{:});
	plot3([1 1 -1 -1 1].*(.262/2), [1 -1 -1 1 1].*((.302/2)-.124), [0 0 0 0 0],lopts{:});
	
	drawnow;
	
	vidObj = play_poly_p1_30fps(C,tf,r,vidObj);
end

if(vid)
	for i = 1:30
		writeVideo(vidObj, getframe(gcf));
	end
	close(vidObj);
end
