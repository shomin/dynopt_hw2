load p2.mat
load c_trajs.mat
%%
figure(3); clf;
subplot 122
cmap = cool(size(c_trajs,3))
for i=1:size(c_trajs,3)
	plot(c_trajs(:,1,i),c_trajs(:,2,i),'-','Color',cmap(i,:),'LineWidth',2);
	hold on;
end
axis equal
grid on

%%

vid = 0;
if(vid)
 vidObj = VideoWriter('dynopt_hw2_mov3.avi');
 vidObj.Quality = 100;
 vidObj.FrameRate = 30;
 open(vidObj);
end



tf = 1;
n = 20;

starts = [.40 .45; .35 -.35; -.2 -.25; -.15 .25; -.26 .45; .05 .42; .42 .05];

% cmap = jet(size(starts,1));
cmap = .1.*ones(size(starts,1),3);
lcmap = cmap;
if(max(max(lcmap))>1)
	lcmap = (lcmap+.01) ./ max(max((lcmap+.01)));
end
lcmap = lcmap .^ 0.3;

grid_pts = cell2mat(data(:,1:2));

for i = 1:size(starts,1)
	
	
	x_start = zeros(28,1);%.01*ones(28,1);
	
	x_start(3) = starts(i,1);
	x_start(4) = starts(i,2);
	
	
	tf = 1;
	n = 20;
	
	%get distances to the points we have trajectories for
	dists = repmat(starts(i,:),size(grid_pts,1),1) - grid_pts;
	sq_dists = sum(dists'.^2);
	
	%sort them
	[~,inds] = sort(sq_dists);
	
	X = zeros(size(data{1,3}));
	
	%weighted average of closest 4 trajectories
	subplot 122
	for j = 1:4
		w(j) = sq_dists(inds(j)) / sum(sq_dists(inds(1:4)))
		X = X + (data{inds(j),3}.*w(j));
		hold on
		plot(c_trajs(1,1,inds(j)),c_trajs(1,2,inds(j)),...
			'o','Color',lcmap(i,:),'LineWidth',2,'MarkerSize',8);
	end


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
	
	
	speed = .3; %play at regular speed
	if(vid)
		[vidObj, com_traj] = play_poly_p2_30fps(C,tf,speed,r,cmap,i,vidObj);
	else
		[~, com_traj] = play_poly_p2_30fps(C,tf,speed,r,cmap,i);
	end
	

end

if(vid)
	for i = 1:30
		writeVideo(vidObj, getframe(gcf));
	end
	close(vidObj);
end
