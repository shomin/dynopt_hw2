function r=play_poly(C,tf,sf,r)

if(nargin<4)
    r = init_robot();
    drawnow;
end

tic
t = toc*sf;
ph = plot3(0,0,0,'.--','MarkerSize',25,'LineWidth',2,'Color',[.2 .9 .3]);
while t < tf
    %t_pow = [1 t t^2 t^3 t^4 t^5]';
    t_pow = [t^5 t^4 t^3 t^2 t 1]';
    th = C*t_pow;
    com = set_robot(r,th);
    set(ph,'xdata',[com(1) com(1)],...
        'ydata',[com(2) com(2)],'zdata',[com(3) 0]);
    drawnow
    t = toc*sf;
end
