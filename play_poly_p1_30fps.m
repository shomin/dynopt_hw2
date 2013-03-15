function vidObj=play_poly_p2_30fps(C,tf,r, vidObj)

if(nargin<4)
	vid = 0;
	vidObj = [];
else
	vid = 1;
end


ph = plot3(0,0,0,'.--','MarkerSize',25,'LineWidth',2,'Color',[.2 .9 .3]);



t = 0;
dt = 1/90;
while t < tf
    %t_pow = [1 t t^2 t^3 t^4 t^5]';
    t_pow = [t^5 t^4 t^3 t^2 t 1]';
    th = C*t_pow;
    com = set_robot(r,th);
    set(ph,'xdata',[com(1) com(1)],...
        'ydata',[com(2) com(2)],'zdata',[com(3) 0]);
    drawnow
	
	if(vid)
		writeVideo(vidObj, getframe(gcf));
	end	
	
    t = t+dt;
end

