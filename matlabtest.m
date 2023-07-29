clear
params.N = 2e5;
params.dt = 250;
params.eps = 1e-6;
params.mu = 3.986004418e5*(1e3)^3;
params.rad = 6378.14e3;
params.deg = 8;
% x0 = [params.rad+100e3; 0;0;0;sqrt(params.mu/(params.rad+100e3));0];
theta = .5;
v0 = 7.350157059479294e+03 * 1.25;
x0 = [
    params.rad + 1000e3;
    0.;
    0.;
    0.;
    v0*sin(theta);
    v0*cos(theta);
    ];


% % dp stuff
x(:,1) = x0;
t = 0; tend = 2*pi*sqrt((params.rad + 1000e3)^3/params.mu) * 20; % 2 obits
tspan = linspace(0,tend,params.N);

% rkstuff
x = zeros(6,params.N);
x(:,1) = x0;
% tic
% for i = 2:length(tspan)
%     x(:,i) = rk4(tspan(i),x(:,i-1),params);
% end
% toc
opts = odeset(AbsTol=params.eps,RelTol=params.eps);

% tic
% [t1,x1,e] = dp45(@(t,x) eoms(t,x,params), x0,[0,tend], params);
% toc
params.omega = 7.292115e-5; % rad/s
opts = odeset("AbsTol",1e-8,"RelTol",1e-8);
tic
[t,x] = ode45(@(t,x) eoms(t,x,params), tspan, x0,opts);
for i = 1:size(x,1)
    comg = cos(params.omega * t(i)); somg = sin(params.omega * t(i));
    params.C_ecef_eci = [comg somg 0; -somg comg 0; 0 0 1];
    xecef(i,1:3) = params.C_ecef_eci * x(i,1:3)';
end
toc

%% plot orbit(s)
f1 = figure(1);
plot3(xecef(:,1),xecef(:,2),xecef(:,3)) % ode
hold on
plot3(x(:,1),x(:,2),x(:,3)) % ode
legend("ECEF Trajectory","ECI Trajectory",'Location','best')

% plot3(x1(1,:),x1(2,:),x1(3,:),'r.') % rk
grid on
axis equal
lims = 3*[-1 1]*params.rad;
dthist = diff(t);
% figure(2)
% plot(t(1:end-1),dthist)
% figure(3)
% plot(t,e)

%% plot sphere
[x,y,z] = sphere(100);
hSurface=surf(x*params.rad,y*params.rad,z*params.rad);
set(hSurface,'FaceColor',[1 1 2]/2, ...
    'FaceAlpha',1,'EdgeColor','none','FaceLighting','gouraud')
camlight
xlim(lims);ylim(lims);zlim(lims)
hold off
%% plot ground tracks
f2 = figure(2);
starttime = datetime(2020,5,10);
stoptime = starttime + seconds(t(end));
sampletime = params.dt;
lla = ecef2lla(xecef);
plot(lla(:,2),lla(:,1))
grid on
xlabel('longitude');ylabel('latitude')
%% Functions


function dxdt = eoms(t,x,params)
% r = norm(x(1:3));
% dxdt(1:3,1) = x(4:6);
% dxdt(4:6,1) = - x(1:3) * params.mu  / r^3;
% comg = cos(params.omega * t); somg = sin(params.omega * t);
% C = [comg -somg 0; somg comg 0; 0 0 1];

dxdt(1:3,1) = [x(4:6)];
dxdt(4:6,1) = -x(1:3) * params.mu/norm(x(1:3))^3;
% [g(1), g(2), g(3)] = gravitysphericalharmonic((C*x(1:3))',params.deg);
% g = C' * g';
% dxdt(4:6,1) = g';
end
function x = rk4(t,x,params)
h = params.dt;
k1 = eoms(t,x,params);
k2 = eoms(t+h/2,x+k1/2*h,params);
k3 = eoms(t+h/2,x+k2/2*h,params);
k4 = eoms(t+h,x+k3*h,params);
x = x + h/6 * (k1 + 2*k2 + 2*k3 + k4);
end

function [t,x,errhistory] = dp45(fun,x0, tspan, params)
h = params.dt;
t = tspan(1);
step_iter = 1;
i = 1;
x(:,1) = x0;
while t < tspan(end)
    k1 = h * fun(t, x(:,i));
    k2 = h * fun(t+h/5, x(:,i)+k1/5);
    k3 = h * fun(t+h*3/10, x(:,i)+k1*3/40 + k2*9/40);
    k4 = h * fun(t+h*4/5, x(:,i) + k1*44/45 - k2*56/15 + k3*32/9);
    k5 = h * fun(t+h*8/9, x(:,i) + k1*19372/6561 - k2*25360/2187 + k3*64448/6561 - k4*212/729);
    k6 = h * fun(t+h, x(:,i) + k1*9017/3168 - k2*355/33 + k3*46732/5247 + k4*49/176 - k5*5103/18656);
    k7 = h * fun(t+h, x(:,i) + k1*35/384 + k3*500/1113 + k4*125/192 - k5*2187/6784 + k6*11/84);

    Z = x(:,i) + 35/384*k1 + 500/1113*k3 + 125/192*k4 - 2187/6784*k5 + ...
        11/84*k6;
    x(:,i+1) = x(:,i) + 5179/57600*k1 + 7571/16695*k3 + 393/640*k4 - ...
        92097/339200*k5 + 187/2100*k6 + 1/40*k7;
    err  = norm(x(:,i+1) - Z);

    if err == 0
        h = h;
    elseif err < params.eps
    % if err < params.eps
        h = 0.9 * h * (params.eps / err)^(1/5);
        step_iter = 1;
    elseif step_iter < 10
        h = 0.9 * h * (params.eps / err)^(1/5);
        step_iter = step_iter + 1;
        continue;
    else
        error("Step size iterations reached max step size calculation attempts")
    end

    t(i+1) = t(i) + h;
    errhistory(i+1) = err;
    i = i+1;
end
end
