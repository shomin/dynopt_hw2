load p2.mat

%%

vid = 0;
if(vid)
 vidObj = VideoWriter('dynopt_hw2_mov2.avi');
 vidObj.Quality = 100;
 vidObj.FrameRate = 30;
 open(vidObj);
end

cmap = jet(size(data,1));

tf = 1;
n = 20;

cnt = 1;
for i = -.5:.1:.5
    for j = -.5:.1:.5
        i
		data{cnt,1}
		j
		data{cnt,2}
		
        x_start = zeros(28,1);%.01*ones(28,1);

        x_start(3) = i;
        x_start(4) = j;


        tf = 1;
        n = 20;
		
		X = data{cnt,3};


	th_0 = x_start;
	thd_0 = zeros(28,1);
	thdd_0 = zeros(28,1);
	
	th_f = zeros(28,1);
	thd_f = zeros(28,1);
	thdd_f = zeros(28,1);
	
	thdd_0 = X(1:28);
	thdd_f = X(29:56);
	
	
	[C, Cd, Cdd] = robot_spline(th_0,thd_0,thdd_0,th_f,thd_f,thdd_f, tf);
	
	
	subplot 121
	hold off
	r = init_robot();
	lopts ={'k-','LineWidth',2};
	plot3([1 1 -1 -1 1].*(.262/2), [1 -1 -1 1 1].*(.302/2), [0 0 0 0 0],lopts{:});
	plot3([1 1 -1 -1 1].*(.262/2), [1 -1 -1 1 1].*((.302/2)-.124), [0 0 0 0 0],lopts{:});
	drawnow;
	
	
	speed = 1; %play at regular speed
	if(vid)
		[vidObj, com_traj] = play_poly_p2_30fps(C,tf,speed,r,cmap,cnt,vidObj);
	else
		[~, com_traj] = play_poly_p2_30fps(C,tf,speed,r,cmap,cnt);
	end
	
	c_trajs(:,:,cnt) = com_traj;
	
	cnt = cnt+1;
	end
end

if(vid)
	for i = 1:30
		writeVideo(vidObj, getframe(gcf));
	end
	close(vidObj);
end
