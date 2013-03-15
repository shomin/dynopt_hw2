function [vidObj, com_traj]=play_poly_p2_30fps(C,tf,sf,r,cmap,ind, vidObj)

if(nargin<4)
    r = init_robot();
    drawnow;
end

if(nargin< 7)
	vid = 0;
	vidObj = [];
else
	vid = 1;
end


ph = plot3(0,0,0,'.--','MarkerSize',25,'LineWidth',2,'Color',[.2 .9 .3]);

th = C*[0 0 0 0 0 1]';
zcom = set_robot(r,zeros(28,1));
com = set_robot(r,th);

subplot 122
plot(zcom(2),-zcom(1),'ko','LineWidth',2); hold on
plot(com(2),-com(1),'o','Color',cmap(ind,:),'MarkerSize',8,'LineWidth',2);
com_h = plot(com(2),-com(1),'-','Color',cmap(ind,:),'LineWidth',2);
axis equal
axis([-.07 .07 -.05 .11]);
grid on




t = 0;
dt = 1/30;
while t < tf
    %t_pow = [1 t t^2 t^3 t^4 t^5]';
    t_pow = [t^5 t^4 t^3 t^2 t 1]';
    th = C*t_pow;
    com = set_robot(r,th);
    set(ph,'xdata',[com(1) com(1)],...
        'ydata',[com(2) com(2)],'zdata',[com(3) 0]);
	set(com_h,'xdata',[get(com_h,'xdata') com(2)],...
		'ydata',[get(com_h,'ydata') -com(1)]);
    drawnow
	
	if(vid)
		writeVideo(vidObj, getframe(gcf));
	end	
	
    t = t+dt;
end

com_traj = [get(com_h,'xdata')' get(com_h,'ydata')'];

