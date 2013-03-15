function r = init_robot()

cyl_n = 20;
cyl_col = [0.7 .1 0.1];
cyl_lines = 0;
cyl_alpha = .4;
cyl_r = .035;
cyl_opts = {cyl_r,cyl_n,cyl_col,cyl_lines,cyl_alpha};


th = zeros(28,1);
thd = zeros(28,1);
thdd = zeros(28,1);
[torqs, pos, lf,lt,rf,rt] = atlas(th,thd,thdd);

r = atlas_data();

for i=2:29
    par = r(i).parent+1;
    chl = r(i).index+1;
    r(i).h = cylinder3(pos(:,par),pos(:,chl),cyl_opts{:});
    hold on;
end

axis equal
axis([-1 1 -1 1 0 1.5])


