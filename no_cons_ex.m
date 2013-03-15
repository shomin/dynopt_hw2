
th_0 = zeros(28,1);
thd_0 = zeros(28,1);
thdd_0 = zeros(28,1);

th_f = .4.*ones(28,1);
thd_f = zeros(28,1);
thdd_f = zeros(28,1);


tf = 1;

H = [1 0 0 -10 15 -6;...
    0 1 0 -6 8 -3;...
    0 0 .5 -1.5 1.5 -.5;...
    0 0 0 10 -15 6;...
    0 0 0 -4 7 -3;...
    0 0 0 .5 -1 .5];

H = H(:,6:-1:1); % flip for MATLAB polynomial convention

C = th_0*H(1,:) + thd_0*H(2,:) + thdd_0*H(3,:) + ...
    th_f*H(4,:) + thd_f*H(5,:) + thdd_f*H(6,:);

Cd = C.*repmat(  [ 5  4 3 2 1 0], 28, 1);
Cdd = C.*repmat( [20 12 6 2 0 0], 28, 1);



%Ht = H.*repmat(t_pow,6,1);
%t_pow = [1 t t^2 t^3 t^4 t^5];

%%

r = init_robot();
%%
tic

sf = .5; %scale factor (1/2 real time)

t = toc*sf;
while t < tf
    %t_pow = [1 t t^2 t^3 t^4 t^5]';
    t_pow = [t^5 t^4 t^3 t^2 t 1]';
    th = C*t_pow;
    set_robot(r,th);
    drawnow
    t = toc*sf;
end