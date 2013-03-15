function com = set_robot(r,th)

thd = zeros(28,1);
thdd = zeros(28,1);

[~,pos,~,~,~,~,~,~,com] = atlas(th,thd,thdd);

cyl_opts = {.035,20,[0.7 .1 0.1],0,.4};

for i=2:29
    par = r(i).parent+1;
    chl = r(i).index+1;
    cylinder3(pos(:,par),pos(:,chl),cyl_opts{:},r(i).h);
end
