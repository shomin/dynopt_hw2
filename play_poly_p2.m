function r=play_poly_p2(C,tf,sf,r)

if(nargin<4)
    r = init_robot();
    drawnow;
end


ph = plot3(0,0,0,'.--','MarkerSize',25,'LineWidth',2,'Color',[.2 .9 .3]);

th = C*[0 0 0 0 0 1]';
zcom = set_robot(r,zeros(28,1));
com = set_robot(r,th);

subplot 122
plot(zcom(1),zcom(2),'ko'); hold on
plot(com(1),com(2),'bo');
com_h = plot(com(1),com(2),'r-');

tic
t = toc*sf;
while t < tf
    %t_pow = [1 t t^2 t^3 t^4 t^5]';
    t_pow = [t^5 t^4 t^3 t^2 t 1]';
    th = C*t_pow;
    com = set_robot(r,th);
    set(ph,'xdata',[com(1) com(1)],...
        'ydata',[com(2) com(2)],'zdata',[com(3) 0]);
	set(com_h,'xdata',[get(com_h,'xdata') com(1)],...
		'ydata',[get(com_h,'ydata') com(2)]);
    drawnow
    t = toc*sf;
end
